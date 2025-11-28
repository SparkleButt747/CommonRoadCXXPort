import { SimulationTelemetryState, SafetyStage } from '../telemetry/index.js';
import type { BackendSnapshot, SimulationBackend } from './backend.js';
import { ModelType } from './types.js';
import { VehicleParameters } from '../models/types.js';
import { VehicleSimulator } from './VehicleSimulator.js';
import { buildModelInterface } from './modelInterface.js';
import { LowSpeedSafety, LowSpeedSafetyConfig, LowSpeedIndices } from './LowSpeedSafety.js';
import { LossOfControlConfig, LossOfControlDetector } from './LossOfControlDetector.js';

export interface JsBackendOptions {
  model: ModelType;
  params: VehicleParameters;
  lowSpeed: LowSpeedSafetyConfig;
  lossConfig: LossOfControlConfig;
  driftEnabled: boolean;
}

interface TelemetryInputs {
  accel: number;
  steerRate: number;
  lateralAccel: number;
  slipRatios: number[];
  frontSlip: number;
  rearSlip: number;
}

export class JsSimulationBackend implements SimulationBackend {
  private params: VehicleParameters;
  private safety: LowSpeedSafety;
  private loss: LossOfControlDetector;
  private simulator: VehicleSimulator;
  private telemetry = new SimulationTelemetryState();
  private dt = 0.01;
  private simTime = 0;
  private distance = 0;
  private energy = 0;
  private indices: LowSpeedIndices;
  ready: Promise<void> = Promise.resolve();

  constructor(private readonly options: JsBackendOptions) {
    this.params = options.params;
    this.indices = buildSafetyIndices(options.model, options.params);
    this.safety = new LowSpeedSafety(options.lowSpeed, this.indices);
    this.loss = new LossOfControlDetector(options.lossConfig);
    this.simulator = new VehicleSimulator(buildModelInterface(options.model), this.params, this.dt, this.safety);
    this.safety.setDriftEnabled(options.driftEnabled);
  }

  reset(initial: number[], dt: number): void {
    this.dt = dt;
    this.indices = buildSafetyIndices(this.options.model, this.params);
    this.safety = new LowSpeedSafety(this.options.lowSpeed, this.indices);
    this.safety.setDriftEnabled(this.options.driftEnabled);
    this.loss.reset();
    this.simulator = new VehicleSimulator(buildModelInterface(this.options.model), this.params, dt, this.safety);
    this.simulator.reset(initial);
    this.telemetry = new SimulationTelemetryState();
    this.simTime = 0;
    this.distance = 0;
    this.energy = 0;
    this.updateTelemetry(this.simulator.currentState(), {
      accel: 0,
      steerRate: 0,
      lateralAccel: 0,
      slipRatios: this.computeSlipRatios(this.simulator.currentState(), 0),
      frontSlip: 0,
      rearSlip: 0,
    });
  }

  step(control: number[], dt: number): void {
    const steerRate = control[0] ?? 0;
    const accel = control[1] ?? 0;
    this.dt = dt ?? this.dt;
    this.simulator.setDt(this.dt);
    this.simulator.step([steerRate, accel]);

    const state = this.simulator.currentState();
    const speed = this.simulator.speed();

    const lateralAccel = this.estimateLateralAccel(state);
    const slipRatios = this.computeSlipRatios(state, speed);
    const yawRate = this.indexValue(state, this.indices.yawRateIndex);
    const slip = this.indexValue(state, this.indices.slipIndex);
    const severity = this.loss.update(this.dt, yawRate, slip, lateralAccel, slipRatios);

    this.simTime += this.dt;
    this.distance += Math.abs(speed) * this.dt;
    this.energy += accel * speed * this.dt;

    const slips = this.estimateAxleSlips(state);

    this.updateTelemetry(state, {
      accel,
      steerRate,
      lateralAccel,
      slipRatios,
      frontSlip: slips.front,
      rearSlip: slips.rear,
    }, severity);
  }

  snapshot(): BackendSnapshot {
    return { state: [...this.simulator.currentState()], telemetry: this.telemetry, dt: this.dt, simulation_time_s: this.simTime };
  }

  speed(): number {
    return this.simulator.speed();
  }

  private estimateAxleSlips(state: number[]): { front: number; rear: number } {
    switch (this.options.model) {
      case ModelType.MB: {
        const yawRate = this.indexValue(state, this.indices.yawRateIndex);
        const vLong = this.indexValue(state, this.indices.longitudinalIndex);
        const vLat = this.indexValue(state, this.indices.lateralIndex);
        const steering = this.indexValue(state, this.indices.steeringIndex);
        const front = Math.atan2(vLat + this.params.a * yawRate, Math.max(Math.abs(vLong), 1e-6)) - steering;
        const rear = Math.atan2(vLat - this.params.b * yawRate, Math.max(Math.abs(vLong), 1e-6));
        return { front, rear };
      }
      case ModelType.ST:
      case ModelType.STD: {
        const speed = this.indexValue(state, this.indices.longitudinalIndex);
        const beta = this.indexValue(state, this.indices.slipIndex);
        const yawRate = this.indexValue(state, this.indices.yawRateIndex);
        const steering = this.indexValue(state, this.indices.steeringIndex);
        const vLong = speed * Math.cos(beta);
        const vLat = speed * Math.sin(beta);
        const front = Math.atan2(vLat + this.params.a * yawRate, Math.max(Math.abs(vLong), 1e-6)) - steering;
        const rear = Math.atan2(vLat - this.params.b * yawRate, Math.max(Math.abs(vLong), 1e-6));
        return { front, rear };
      }
      default:
        return { front: 0, rear: 0 };
    }
  }

  private estimateLateralAccel(state: number[]): number {
    const yawRate = this.indexValue(state, this.indices.yawRateIndex);
    let vLong = this.indexValue(state, this.indices.longitudinalIndex);
    let vLat = 0;
    if (this.indices.lateralIndex !== undefined) {
      vLat = this.indexValue(state, this.indices.lateralIndex);
    } else if (this.indices.slipIndex !== undefined) {
      const slip = this.indexValue(state, this.indices.slipIndex);
      vLat = vLong * Math.tan(slip);
      vLong = vLong / Math.max(Math.cos(slip), 1e-6);
    }
    return yawRate * vLong; // approximate body lateral acceleration
  }

  private computeSlipRatios(state: number[], speed: number): number[] {
    const denom = Math.max(speed, 1e-6);
    if (this.options.model === ModelType.MB) {
      const front = this.indexValue(state, 23);
      const rear = this.indexValue(state, 25);
      return [(front - speed) / denom, (rear - speed) / denom];
    }
    if (this.options.model === ModelType.STD) {
      const front = this.indexValue(state, 7);
      const rear = this.indexValue(state, 8);
      return [(front - speed) / denom, (rear - speed) / denom];
    }
    return [0, 0];
  }

  private indexValue(state: number[], index?: number): number {
    if (index === undefined) return 0;
    if (index < 0 || index >= state.length) return 0;
    const value = state[index];
    return Number.isFinite(value) ? value : 0;
  }

  private updateTelemetry(state: number[], inputs: TelemetryInputs, detectorSeverity = 0): void {
    const wheelRadius = this.params.R_w;
    const frontSplit = clamp(this.params.T_sb, 0, 1);
    const rearSplit = 1 - frontSplit;

    let yaw = 0;
    switch (this.options.model) {
      case ModelType.MB:
        yaw = state[4] ?? 0;
        break;
      case ModelType.ST:
      case ModelType.STD:
      default:
        yaw = state[4] ?? 0;
        break;
    }

    const vLong = this.indexValue(state, this.indices.longitudinalIndex);
    const vLat = this.indexValue(state, this.indices.lateralIndex ?? -1);
    const yawRate = this.indexValue(state, this.indices.yawRateIndex);
    const slipAngle = this.indexValue(state, this.indices.slipIndex);
    const speed = this.simulator.speed();
    const vx = vLong * Math.cos(yaw) - vLat * Math.sin(yaw);
    const vy = vLong * Math.sin(yaw) + vLat * Math.cos(yaw);

    const throttle = inputs.accel > 0 ? inputs.accel / Math.max(this.params.longitudinal.a_max, 1e-6) : 0;
    const brake = inputs.accel < 0 ? -inputs.accel / Math.max(this.params.longitudinal.a_max, 1e-6) : 0;
    const driveForce = Math.max(0, inputs.accel) * this.params.m;
    const brakeForce = Math.max(0, -inputs.accel) * this.params.m;
    const frontDrive = driveForce * frontSplit;
    const rearDrive = driveForce * rearSplit;
    const frontBrake = brakeForce * frontSplit;
    const rearBrake = brakeForce * rearSplit;

    const normalFront = this.params.m * 9.81 * (this.params.b / Math.max(this.params.a + this.params.b, 1e-6));
    const normalRear = this.params.m * 9.81 * (this.params.a / Math.max(this.params.a + this.params.b, 1e-6));
    const maxFront = this.params.tire.p_dy1 * normalFront;
    const maxRear = this.params.tire.p_dy1 * normalRear;

    const telem = new SimulationTelemetryState();
    telem.pose.x = state[0] ?? 0;
    telem.pose.y = state[1] ?? 0;
    telem.pose.yaw = yaw;

    telem.velocity.speed = speed;
    telem.velocity.longitudinal = vLong;
    telem.velocity.lateral = vLat;
    telem.velocity.yaw_rate = yawRate;
    telem.velocity.global_x = vx;
    telem.velocity.global_y = vy;

    telem.acceleration.longitudinal = inputs.accel;
    telem.acceleration.lateral = inputs.lateralAccel;

    telem.traction.slip_angle = slipAngle;
    telem.traction.front_slip_angle = inputs.frontSlip;
    telem.traction.rear_slip_angle = inputs.rearSlip;
    const totalNormal = Math.max(this.params.m * 9.81, 1e-6);
    const lateralAvailable = Math.max(this.params.tire.p_dy1 * totalNormal, 1e-6);
    telem.traction.lateral_force_saturation = Math.min(1, Math.abs(this.params.m * inputs.lateralAccel) / lateralAvailable);
    telem.traction.drift_mode = this.options.driftEnabled;

    telem.steering.desired_angle = state[this.indices.steeringIndex ?? 0] ?? 0;
    telem.steering.actual_angle = telem.steering.desired_angle;
    telem.steering.desired_rate = inputs.steerRate;
    telem.steering.actual_rate = inputs.steerRate;

    telem.controller.acceleration = inputs.accel;
    telem.controller.throttle = throttle;
    telem.controller.brake = brake;
    telem.controller.drive_force = driveForce;
    telem.controller.brake_force = brakeForce;
    telem.controller.regen_force = 0;
    telem.controller.hydraulic_force = brakeForce;
    telem.controller.drag_force = 0;
    telem.controller.rolling_force = 0;

    const driveTorque = driveForce * wheelRadius;
    const brakeTorque = brakeForce * wheelRadius;

    telem.powertrain.drive_torque = driveTorque;
    telem.powertrain.regen_torque = 0;
    telem.powertrain.total_torque = driveTorque - brakeTorque;
    telem.powertrain.mechanical_power = (driveForce - brakeForce) * speed;
    telem.powertrain.battery_power = telem.powertrain.mechanical_power;
    telem.powertrain.soc = this.telemetry.powertrain.soc || 0.5;

    telem.front_axle.drive_torque = driveTorque * frontSplit;
    telem.rear_axle.drive_torque = driveTorque * rearSplit;
    telem.front_axle.brake_torque = brakeTorque * frontSplit;
    telem.rear_axle.brake_torque = brakeTorque * rearSplit;
    telem.front_axle.regen_torque = 0;
    telem.rear_axle.regen_torque = 0;
    telem.front_axle.normal_force = normalFront;
    telem.rear_axle.normal_force = normalRear;

    const frontSlipRatio = inputs.slipRatios[0] ?? 0;
    const rearSlipRatio = inputs.slipRatios[1] ?? 0;

    telem.front_axle.left.speed = this.estimateWheelSpeed(state, true);
    telem.front_axle.right.speed = telem.front_axle.left.speed;
    telem.rear_axle.left.speed = this.estimateWheelSpeed(state, false);
    telem.rear_axle.right.speed = telem.rear_axle.left.speed;

    telem.front_axle.left.slip_ratio = frontSlipRatio;
    telem.front_axle.right.slip_ratio = frontSlipRatio;
    telem.rear_axle.left.slip_ratio = rearSlipRatio;
    telem.rear_axle.right.slip_ratio = rearSlipRatio;

    telem.front_axle.left.friction_utilization = combinedUtilization(frontDrive - frontBrake, inputs.frontSlip, maxFront);
    telem.front_axle.right.friction_utilization = telem.front_axle.left.friction_utilization;
    telem.rear_axle.left.friction_utilization = combinedUtilization(rearDrive - rearBrake, inputs.rearSlip, maxRear);
    telem.rear_axle.right.friction_utilization = telem.rear_axle.left.friction_utilization;

    const safetyStatus = this.safety.status(state, speed);
    telem.detector_severity = Math.max(detectorSeverity, safetyStatus.severity);
    telem.safety_stage = safetyStatus.stage;
    telem.detector_forced = safetyStatus.detector_forced;
    telem.low_speed_engaged = safetyStatus.latch_active;
    telem.traction.drift_mode = safetyStatus.drift_mode;

    telem.totals.distance_traveled_m = this.distance;
    telem.totals.energy_consumed_joules = this.energy;
    telem.totals.simulation_time_s = this.simTime;

    this.telemetry = telem;
  }

  private estimateWheelSpeed(state: number[], front: boolean): number {
    if (this.options.model === ModelType.MB) {
      return front ? this.indexValue(state, 23) : this.indexValue(state, 25);
    }
    if (this.options.model === ModelType.STD) {
      return front ? this.indexValue(state, 7) : this.indexValue(state, 8);
    }
    return this.simulator.speed();
  }
}

function buildSafetyIndices(model: ModelType, params: VehicleParameters): LowSpeedIndices {
  let longitudinalIndex: number | undefined;
  let lateralIndex: number | undefined;
  let yawRateIndex: number | undefined;
  let slipIndex: number | undefined;
  let wheelSpeedIndices: number[] | undefined;
  let steeringIndex: number | undefined;

  switch (model) {
    case ModelType.ST:
      longitudinalIndex = 3;
      yawRateIndex = 5;
      slipIndex = 6;
      steeringIndex = 2;
      break;
    case ModelType.STD:
      longitudinalIndex = 3;
      yawRateIndex = 5;
      slipIndex = 6;
      steeringIndex = 2;
      wheelSpeedIndices = [7, 8];
      break;
    case ModelType.MB:
    default:
      longitudinalIndex = 3;
      lateralIndex = 10;
      yawRateIndex = 5;
      steeringIndex = 2;
      wheelSpeedIndices = [23, 24, 25, 26];
      break;
  }

  const wheelbase = params.a + params.b;
  const rearLength = params.b;

  return {
    longitudinalIndex,
    lateralIndex,
    yawRateIndex,
    slipIndex,
    wheelSpeedIndices,
    steeringIndex,
    wheelbase: wheelbase > 0 ? wheelbase : undefined,
    rearLength: rearLength > 0 ? rearLength : undefined,
  };
}

function clamp(value: number, min: number, max: number): number {
  if (!Number.isFinite(value)) return min;
  return Math.min(Math.max(value, min), max);
}

function combinedUtilization(longitudinalForce: number, lateralForce: number, normalForce: number): number {
  const muForce = Math.max(Math.abs(normalForce), 1e-6);
  return Math.min(1, Math.hypot(longitudinalForce, lateralForce) / muForce);
}
