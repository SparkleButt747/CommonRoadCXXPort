import { SimulationTelemetryState, SafetyStage } from '../telemetry/index.js';
import type { BackendSnapshot, SimulationBackend } from './backend.js';
import { ModelType } from './types.js';

interface SteeringConstraints {
  max: number;
  min: number;
  v_max: number;
  v_min: number;
}

interface LongitudinalConstraints {
  a_max: number;
  v_max: number;
  v_min: number;
}

interface VehicleParameters {
  a?: number;
  b?: number;
  m?: number;
  I_z?: number;
  R_w?: number;
  T_sb?: number;
  mu?: number;
  cornering_stiffness_front?: number;
  cornering_stiffness_rear?: number;
  drag_coefficient?: number;
  rolling_resistance?: number;
  steering?: SteeringConstraints;
  longitudinal?: LongitudinalConstraints;
}

interface LowSpeedProfile {
  engage_speed: number;
  release_speed: number;
  yaw_rate_limit: number;
  slip_angle_limit: number;
}

interface LowSpeedSafetyConfig {
  normal: LowSpeedProfile;
  drift: LowSpeedProfile;
  stop_speed_epsilon: number;
  drift_enabled: boolean;
}

interface LossThreshold {
  threshold: number;
  rate: number;
}

interface LossOfControlConfig {
  yaw_rate?: LossThreshold;
  slip_angle?: LossThreshold;
  lateral_accel?: LossThreshold;
  slip_ratio?: LossThreshold;
}

interface LossMetricState {
  value: number;
  hasPrevious: boolean;
}

export interface JsBackendOptions {
  model: ModelType;
  params: VehicleParameters;
  lowSpeed: LowSpeedSafetyConfig;
  lossConfig: LossOfControlConfig;
  driftEnabled: boolean;
}

class LossOfControlDetector {
  private yawRate: LossMetricState = { value: 0, hasPrevious: false };
  private slipAngle: LossMetricState = { value: 0, hasPrevious: false };
  private lateralAccel: LossMetricState = { value: 0, hasPrevious: false };
  private wheels: LossMetricState[] = [];
  private severity = 0;

  constructor(private readonly config: LossOfControlConfig) {}

  reset(): void {
    this.yawRate = { value: 0, hasPrevious: false };
    this.slipAngle = { value: 0, hasPrevious: false };
    this.lateralAccel = { value: 0, hasPrevious: false };
    this.wheels = [];
    this.severity = 0;
  }

  update(dt: number, yawRate: number, slipAngle: number, lateralAccel: number, wheelSlipRatios: number[]): number {
    if (!(dt > 0)) return this.severity;
    if (this.wheels.length !== wheelSlipRatios.length) {
      this.wheels = wheelSlipRatios.map(() => ({ value: 0, hasPrevious: false }));
    }

    const scores = [
      this.evaluateMetric(dt, yawRate, this.yawRate, this.config.yaw_rate),
      this.evaluateMetric(dt, slipAngle, this.slipAngle, this.config.slip_angle),
      this.evaluateMetric(dt, lateralAccel, this.lateralAccel, this.config.lateral_accel),
    ];

    wheelSlipRatios.forEach((ratio, idx) => {
      scores.push(this.evaluateMetric(dt, ratio, this.wheels[idx], this.config.slip_ratio));
    });

    this.severity = Math.max(0, ...scores);
    return this.severity;
  }

  private evaluateMetric(dt: number, value: number, state: LossMetricState, limits?: LossThreshold): number {
    if (!limits) {
      state.value = value;
      state.hasPrevious = true;
      return 0;
    }
    let severity = 0;
    if (state.hasPrevious) {
      const delta = Math.abs(value - state.value);
      const rate = delta / Math.max(dt, 1e-9);
      const mag = Math.abs(value);
      if (mag >= limits.threshold && rate >= limits.rate) {
        const magScore = (mag - limits.threshold) / limits.threshold;
        const rateScore = (rate - limits.rate) / limits.rate;
        severity = Math.max(0, 0.5 * magScore + 0.5 * rateScore);
      }
    }
    state.value = value;
    state.hasPrevious = true;
    return severity;
  }

  getSeverity(): number {
    return this.severity;
  }
}

class LowSpeedSafety {
  private engaged = false;
  private driftEnabled: boolean;

  constructor(private readonly config: LowSpeedSafetyConfig, driftEnabled: boolean) {
    this.driftEnabled = driftEnabled;
  }

  reset(): void {
    this.engaged = false;
  }

  setDriftEnabled(enabled: boolean): void {
    this.driftEnabled = enabled;
  }

  status(
    state: number[],
    speed: number
  ): { severity: number; stage: SafetyStage; detectorForced: boolean; latchActive: boolean; driftMode: boolean } {
    const profile = this.config.active_profile ? this.config.active_profile(this.driftEnabled) : this.profile();
    const yaw = state[5] ?? 0;
    const slip = state[6] ?? 0;
    const yawRatio = Math.abs(yaw) / Math.max(profile.yaw_rate_limit, 1e-6);
    const slipRatio = Math.abs(slip) / Math.max(profile.slip_angle_limit, 1e-6);
    const severity = Math.max(yawRatio, slipRatio);
    const detectorForced = severity > 1.0;
    const transitionBlend = this.preLatchBlend(speed, profile);
    const latchActive = this.engaged || detectorForced;
    const stage = latchActive
      ? SafetyStage.Emergency
      : transitionBlend > 0
        ? SafetyStage.Transition
        : SafetyStage.Normal;
    return { severity, stage, detectorForced, latchActive, driftMode: this.driftEnabled };
  }

  apply(state: number[], speed: number, yawRateIdx: number, slipIdx: number, lateralIdx: number, steeringIdx: number, wheelBase: number, rearLength: number): void {
    const profile = this.profile();
    const yaw = state[yawRateIdx] ?? 0;
    const slip = state[slipIdx] ?? 0;
    const severity = Math.max(
      Math.abs(yaw) / Math.max(profile.yaw_rate_limit, 1e-6),
      Math.abs(slip) / Math.max(profile.slip_angle_limit, 1e-6)
    );

    const severityTrip = severity > 1.0;
    if (this.engaged) {
      if (speed > profile.release_speed && !severityTrip) {
        this.engaged = false;
      }
    } else if (speed < profile.engage_speed || severityTrip) {
      this.engaged = true;
    }

    const transitionBlend = this.preLatchBlend(speed, profile);
    const latchActive = this.engaged || severityTrip;
    const allowUnclamped = this.driftEnabled && !latchActive && transitionBlend === 0;

    if (!allowUnclamped) {
      const yawLimit = latchActive ? profile.yaw_rate_limit : this.scaledLimit(profile.yaw_rate_limit, speed, profile);
      state[yawRateIdx] = Math.max(-yawLimit, Math.min(yawLimit, state[yawRateIdx] ?? 0));

      const slipLimit = latchActive ? profile.slip_angle_limit : this.scaledLimit(profile.slip_angle_limit, speed, profile);
      const targetSlip = this.targetSlip(state, speed, steeringIdx, wheelBase, rearLength, slipLimit, latchActive);
      state[slipIdx] = (1 - transitionBlend) * (state[slipIdx] ?? 0) + transitionBlend * targetSlip;

      if (Math.abs(state[lateralIdx] ?? 0) <= this.config.stop_speed_epsilon) {
        state[lateralIdx] = 0;
      }
    }
  }

  private profile(): LowSpeedProfile {
    return this.config.drift_enabled ? this.config.drift : this.config.normal;
  }

  private preLatchBlend(speed: number, profile: LowSpeedProfile): number {
    const band = Math.max(profile.release_speed - profile.engage_speed, 0);
    const upper = profile.release_speed + band;
    const lower = profile.release_speed;
    if (upper <= lower || speed >= upper) return 0;
    if (speed <= lower) return 1;
    const ratio = (upper - speed) / (upper - lower);
    return Math.max(0, Math.min(1, ratio));
  }

  private scaledLimit(limit: number, speed: number, profile: LowSpeedProfile): number {
    if (speed >= profile.release_speed || profile.release_speed <= 0) return limit;
    const ratio = Math.max(0, Math.min(1, speed / Math.max(profile.release_speed, 1e-6)));
    const minLimit = Math.max(this.config.stop_speed_epsilon, 1e-6);
    return Math.max(minLimit, Math.min(limit, limit * ratio));
  }

  private targetSlip(
    state: number[],
    speed: number,
    steeringIdx: number,
    wheelBase: number,
    rearLength: number,
    limit: number,
    emergency: boolean
  ): number {
    const steering = state[steeringIdx] ?? 0;
    const beta = wheelBase > 0 ? Math.atan(Math.tan(steering) * (rearLength / wheelBase)) : 0;
    const target = Math.max(-limit, Math.min(limit, beta));
    if (emergency && Math.abs(speed) <= this.config.stop_speed_epsilon) {
      return 0;
    }
    return target;
  }
}

export class JsSimulationBackend implements SimulationBackend {
  private state: number[] = [];
  private dt = 0.01;
  private telemetry = new SimulationTelemetryState();
  private simTime = 0;
  private distance = 0;
  private energy = 0;
  private safety: LowSpeedSafety;
  private loss: LossOfControlDetector;
  private params: NormalizedParameters;
  private wheelBase: number;
  ready: Promise<void> = Promise.resolve();

  constructor(private readonly options: JsBackendOptions) {
    this.params = normalizeParameters(options.model, options.params);
    this.wheelBase = this.params.a + this.params.b;
    this.safety = new LowSpeedSafety(options.lowSpeed, options.driftEnabled);
    this.loss = new LossOfControlDetector(options.lossConfig);
  }

  reset(initial: number[], dt: number): void {
    this.params = normalizeParameters(this.options.model, this.options.params);
    this.wheelBase = this.params.a + this.params.b;
    this.dt = dt;
    this.simTime = 0;
    this.distance = 0;
    this.energy = 0;
    this.safety.reset();
    this.loss.reset();
    this.state = this.initializeState(initial);
    this.telemetry = new SimulationTelemetryState();
    this.updateTelemetry({
      accel: 0,
      steerRate: 0,
      frontSlip: 0,
      rearSlip: 0,
      slipRatios: [0, 0],
      driveForce: 0,
      brakeForce: 0,
      frontDrive: 0,
      rearDrive: 0,
      frontBrake: 0,
      rearBrake: 0,
      lateralAccel: 0,
      yawAccel: 0,
      throttle: 0,
      brake: 0,
      frontCombinedUtil: 0,
      rearCombinedUtil: 0,
    });
  }

  step(control: number[], dt: number): void {
    const steerRateCmd = control[0] ?? 0;
    const accelCmd = control[1] ?? 0;
    const steering = this.params.steering;
    const longitudinal = this.params.longitudinal;

    const steerRate = clamp(steerRateCmd, steering.v_min, steering.v_max);
    const accel = clamp(accelCmd, -longitudinal.a_max, longitudinal.a_max);

    const dtClamped = dt ?? this.dt;
    this.dt = dtClamped;

    const yaw = this.state[2];
    let steeringAngle = clamp(this.state[7] + steerRate * dtClamped, steering.min, steering.max);

    const vLong = clamp(this.state[3], longitudinal.v_min ?? -50, longitudinal.v_max ?? 50);
    const vLat = this.state[4];
    const yawRate = this.state[5];

    const frontSlip = Math.atan2(vLat + this.params.a * yawRate, Math.max(Math.abs(vLong), 1e-3)) - steeringAngle;
    const rearSlip = Math.atan2(vLat - this.params.b * yawRate, Math.max(Math.abs(vLong), 1e-3));

    const normalFront = this.params.m * 9.81 * (this.params.b / Math.max(this.wheelBase, 1e-6));
    const normalRear = this.params.m * 9.81 * (this.params.a / Math.max(this.wheelBase, 1e-6));

    const maxFront = this.params.mu * normalFront;
    const maxRear = this.params.mu * normalRear;
    const FyF = clamp(-this.params.cornering_stiffness_front * frontSlip, -maxFront, maxFront);
    const FyR = clamp(-this.params.cornering_stiffness_rear * rearSlip, -maxRear, maxRear);

    const throttle = accel > 0 ? accel / Math.max(longitudinal.a_max, 1e-6) : 0;
    const brake = accel < 0 ? -accel / Math.max(longitudinal.a_max, 1e-6) : 0;
    const driveForce = Math.max(0, accel) * this.params.m;
    const brakeForce = Math.max(0, -accel) * this.params.m;
    const frontSplit = clamp(this.params.T_sb, 0, 1);
    const frontDrive = driveForce * frontSplit;
    const rearDrive = driveForce * (1 - frontSplit);
    const frontBrake = brakeForce * frontSplit;
    const rearBrake = brakeForce * (1 - frontSplit);

    const FxTotal = frontDrive + rearDrive - (frontBrake + rearBrake)
      - this.params.drag_coefficient * vLong * Math.abs(vLong)
      - this.params.rolling_resistance;
    const vLongDot = FxTotal / Math.max(this.params.m, 1e-6) + yawRate * vLat;
    const vLatDot = (FyF + FyR) / Math.max(this.params.m, 1e-6) - yawRate * vLong;
    const yawAcc = (this.params.a * FyF - this.params.b * FyR) / Math.max(this.params.I_z, 1e-6);

    const nextVLong = clamp(vLong + vLongDot * dtClamped, longitudinal.v_min ?? -50, longitudinal.v_max ?? 50);
    const nextVLat = vLat + vLatDot * dtClamped;
    const nextYawRate = yawRate + yawAcc * dtClamped;
    const nextYaw = yaw + nextYawRate * dtClamped;
    const speed = Math.hypot(nextVLong, nextVLat);

    const globalVx = nextVLong * Math.cos(nextYaw) - nextVLat * Math.sin(nextYaw);
    const globalVy = nextVLong * Math.sin(nextYaw) + nextVLat * Math.cos(nextYaw);

    this.state[0] += globalVx * dtClamped;
    this.state[1] += globalVy * dtClamped;
    this.state[2] = nextYaw;
    this.state[3] = nextVLong;
    this.state[4] = nextVLat;
    this.state[5] = nextYawRate;
    this.state[6] = Math.atan2(nextVLat, Math.max(nextVLong, 1e-6));
    this.state[7] = steeringAngle;
    this.state[8] = Math.max(0, nextVLong + this.params.a * nextYawRate);
    this.state[9] = Math.max(0, nextVLong - this.params.b * nextYawRate);

    const lateralAccel = nextYawRate * nextVLong + vLatDot;

    this.safety.setDriftEnabled(this.options.driftEnabled);
    this.safety.apply(this.state, speed, 5, 6, 4, 7, this.wheelBase, this.params.b);
    const slipRatios = this.computeSlipRatios(speed, this.state[8], this.state[9]);
    const severity = this.loss.update(dtClamped, this.state[5], this.state[6], lateralAccel, slipRatios);

    const frontUtil = combinedUtilization(frontDrive - frontBrake, FyF, maxFront);
    const rearUtil = combinedUtilization(rearDrive - rearBrake, FyR, maxRear);

    this.simTime += dtClamped;
    this.distance += speed * dtClamped;
    this.energy += (driveForce - brakeForce) * speed * dtClamped;

    this.updateTelemetry({
      accel,
      steerRate,
      frontSlip,
      rearSlip,
      slipRatios,
      driveForce,
      brakeForce,
      frontDrive,
      rearDrive,
      frontBrake,
      rearBrake,
      lateralAccel,
      yawAccel,
      throttle,
      brake,
      frontCombinedUtil: frontUtil,
      rearCombinedUtil: rearUtil,
    }, severity);
  }

  snapshot(): BackendSnapshot {
    return { state: [...this.state], telemetry: this.telemetry, dt: this.dt, simulation_time_s: this.simTime };
  }

  speed(): number {
    return Math.abs(this.state[3] ?? 0);
  }

  private initializeState(initial: number[]): number[] {
    const base = new Array(10).fill(0);
    for (let i = 0; i < Math.min(initial.length, base.length); i++) {
      base[i] = initial[i] ?? 0;
    }
    if (!Number.isFinite(base[3])) base[3] = 0;
    const speed = Math.max(0, base[3]);
    if (!Number.isFinite(base[6])) {
      base[6] = Math.abs(speed) > 1e-9 ? Math.atan2(base[4], speed) : 0;
    }
    if (!Number.isFinite(base[8])) base[8] = speed;
    if (!Number.isFinite(base[9])) base[9] = speed;
    return base;
  }

  private computeSlipRatios(speed: number, front: number, rear: number): number[] {
    const denom = Math.max(speed, 1e-6);
    return [(front - speed) / denom, (rear - speed) / denom];
  }

  private updateTelemetry(inputs: TelemetryInputs, detectorSeverity = 0): void {
    const wheelRadius = this.params.R_w;
    const frontSplit = clamp(this.params.T_sb, 0, 1);
    const rearSplit = 1 - frontSplit;

    const speed = Math.max(0, Math.hypot(this.state[3], this.state[4]));
    const yawRate = this.state[5] ?? 0;
    const slipAngle = this.state[6] ?? 0;
    const steeringAngle = this.state[7] ?? 0;
    const vx = this.state[3] * Math.cos(this.state[2]) - this.state[4] * Math.sin(this.state[2]);
    const vy = this.state[3] * Math.sin(this.state[2]) + this.state[4] * Math.cos(this.state[2]);

    const driveTorque = inputs.driveForce * wheelRadius;
    const brakeTorque = inputs.brakeForce * wheelRadius;

    const telem = new SimulationTelemetryState();
    telem.pose.x = this.state[0];
    telem.pose.y = this.state[1];
    telem.pose.yaw = this.state[2];

    telem.velocity.speed = speed;
    telem.velocity.longitudinal = this.state[3];
    telem.velocity.lateral = this.state[4];
    telem.velocity.yaw_rate = yawRate;
    telem.velocity.global_x = vx;
    telem.velocity.global_y = vy;

    telem.acceleration.longitudinal = inputs.accel;
    telem.acceleration.lateral = inputs.lateralAccel;

    telem.traction.slip_angle = slipAngle;
    telem.traction.front_slip_angle = inputs.frontSlip;
    telem.traction.rear_slip_angle = inputs.rearSlip;
    const totalNormalForce = Math.max(this.params.m * 9.81, 1e-6);
    const lateralAvailable = Math.max(this.params.mu * totalNormalForce, 1e-6);
    telem.traction.lateral_force_saturation = Math.min(1, Math.abs(this.params.m * inputs.lateralAccel) / lateralAvailable);
    telem.traction.drift_mode = this.options.driftEnabled;

    telem.steering.desired_angle = steeringAngle;
    telem.steering.actual_angle = steeringAngle;
    telem.steering.desired_rate = inputs.steerRate;
    telem.steering.actual_rate = inputs.steerRate;

    telem.controller.acceleration = inputs.accel;
    telem.controller.throttle = inputs.throttle;
    telem.controller.brake = inputs.brake;
    telem.controller.drive_force = inputs.driveForce;
    telem.controller.brake_force = inputs.brakeForce;
    telem.controller.regen_force = 0;
    telem.controller.hydraulic_force = inputs.brakeForce;
    telem.controller.drag_force = this.params.drag_coefficient * speed * speed;
    telem.controller.rolling_force = this.params.rolling_resistance;

    telem.powertrain.drive_torque = driveTorque;
    telem.powertrain.regen_torque = 0;
    telem.powertrain.total_torque = driveTorque - brakeTorque;
    telem.powertrain.mechanical_power = (inputs.driveForce - inputs.brakeForce) * speed;
    telem.powertrain.battery_power = telem.powertrain.mechanical_power;
    telem.powertrain.soc = this.telemetry.powertrain.soc || 0.5;

    telem.front_axle.drive_torque = driveTorque * frontSplit;
    telem.rear_axle.drive_torque = driveTorque * rearSplit;
    telem.front_axle.brake_torque = brakeTorque * frontSplit;
    telem.rear_axle.brake_torque = brakeTorque * rearSplit;
    telem.front_axle.regen_torque = 0;
    telem.rear_axle.regen_torque = 0;
    telem.front_axle.normal_force = this.params.m * 9.81 * (this.params.b / Math.max(this.wheelBase, 1e-6));
    telem.rear_axle.normal_force = this.params.m * 9.81 * (this.params.a / Math.max(this.wheelBase, 1e-6));

    telem.front_axle.left.speed = this.state[8];
    telem.front_axle.right.speed = this.state[8];
    telem.rear_axle.left.speed = this.state[9];
    telem.rear_axle.right.speed = this.state[9];

    telem.front_axle.left.slip_ratio = inputs.slipRatios[0];
    telem.front_axle.right.slip_ratio = inputs.slipRatios[0];
    telem.rear_axle.left.slip_ratio = inputs.slipRatios[1];
    telem.rear_axle.right.slip_ratio = inputs.slipRatios[1];

    telem.front_axle.left.friction_utilization = inputs.frontCombinedUtil;
    telem.front_axle.right.friction_utilization = inputs.frontCombinedUtil;
    telem.rear_axle.left.friction_utilization = inputs.rearCombinedUtil;
    telem.rear_axle.right.friction_utilization = inputs.rearCombinedUtil;

    const safetyStatus = this.safety.status(this.state, speed);
    telem.detector_severity = Math.max(detectorSeverity, safetyStatus.severity);
    telem.safety_stage = safetyStatus.stage;
    telem.detector_forced = safetyStatus.detectorForced;
    telem.low_speed_engaged = safetyStatus.latchActive;
    telem.traction.drift_mode = safetyStatus.driftMode;

    telem.totals.distance_traveled_m = this.distance;
    telem.totals.energy_consumed_joules = this.energy;
    telem.totals.simulation_time_s = this.simTime;

    this.telemetry = telem;
  }
}

interface NormalizedParameters {
  a: number;
  b: number;
  m: number;
  I_z: number;
  R_w: number;
  T_sb: number;
  mu: number;
  cornering_stiffness_front: number;
  cornering_stiffness_rear: number;
  drag_coefficient: number;
  rolling_resistance: number;
  steering: SteeringConstraints;
  longitudinal: LongitudinalConstraints;
}

interface TelemetryInputs {
  accel: number;
  steerRate: number;
  frontSlip: number;
  rearSlip: number;
  slipRatios: number[];
  driveForce: number;
  brakeForce: number;
  frontDrive: number;
  rearDrive: number;
  frontBrake: number;
  rearBrake: number;
  lateralAccel: number;
  yawAccel: number;
  throttle: number;
  brake: number;
  frontCombinedUtil: number;
  rearCombinedUtil: number;
}

function normalizeParameters(model: ModelType, overrides: VehicleParameters): NormalizedParameters {
  const baseByModel: Record<ModelType, NormalizedParameters> = {
    [ModelType.MB]: {
      a: 1.5,
      b: 1.5,
      m: 1650,
      I_z: 2600,
      R_w: 0.32,
      T_sb: 0.55,
      mu: 1.0,
      cornering_stiffness_front: 80000,
      cornering_stiffness_rear: 82000,
      drag_coefficient: 0.35,
      rolling_resistance: 12,
      steering: { max: 0.6, min: -0.6, v_max: 0.8, v_min: -0.8 },
      longitudinal: { a_max: 8, v_max: 80, v_min: -10 },
    },
    [ModelType.ST]: {
      a: 1.35,
      b: 1.45,
      m: 1500,
      I_z: 2300,
      R_w: 0.31,
      T_sb: 0.55,
      mu: 1.0,
      cornering_stiffness_front: 76000,
      cornering_stiffness_rear: 78000,
      drag_coefficient: 0.32,
      rolling_resistance: 10,
      steering: { max: 0.55, min: -0.55, v_max: 0.75, v_min: -0.75 },
      longitudinal: { a_max: 7, v_max: 75, v_min: -9 },
    },
    [ModelType.STD]: {
      a: 1.3,
      b: 1.5,
      m: 1550,
      I_z: 2400,
      R_w: 0.31,
      T_sb: 0.55,
      mu: 1.0,
      cornering_stiffness_front: 78000,
      cornering_stiffness_rear: 80000,
      drag_coefficient: 0.34,
      rolling_resistance: 11,
      steering: { max: 0.6, min: -0.6, v_max: 0.75, v_min: -0.75 },
      longitudinal: { a_max: 7.5, v_max: 78, v_min: -9 },
    },
  };

  const base = baseByModel[model];
  const steering = { ...base.steering, ...(overrides.steering ?? {}) };
  const longitudinal = { ...base.longitudinal, ...(overrides.longitudinal ?? {}) };

  return {
    ...base,
    ...overrides,
    steering,
    longitudinal,
    a: overrides.a ?? base.a,
    b: overrides.b ?? base.b,
    m: overrides.m ?? base.m,
    I_z: overrides.I_z ?? base.I_z,
    R_w: overrides.R_w ?? base.R_w,
    T_sb: overrides.T_sb ?? base.T_sb,
    mu: overrides.mu ?? base.mu,
    cornering_stiffness_front: overrides.cornering_stiffness_front ?? base.cornering_stiffness_front,
    cornering_stiffness_rear: overrides.cornering_stiffness_rear ?? base.cornering_stiffness_rear,
    drag_coefficient: overrides.drag_coefficient ?? base.drag_coefficient,
    rolling_resistance: overrides.rolling_resistance ?? base.rolling_resistance,
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

