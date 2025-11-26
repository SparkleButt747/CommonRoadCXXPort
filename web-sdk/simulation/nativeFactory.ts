import { SimulationTelemetryState, SafetyStage } from '../telemetry/index.js';
import { ModelType } from './types.js';
import type { NativeDaemonFactory, NativeDaemonHandle } from './backend.js';

type LossOfControlChannelThresholds = { threshold: number; rate: number };

type LossOfControlConfig = {
  yaw_rate?: LossOfControlChannelThresholds;
  slip_angle?: LossOfControlChannelThresholds;
  lateral_accel?: LossOfControlChannelThresholds;
  slip_ratio?: LossOfControlChannelThresholds;
};

type LowSpeedRegimeConfig = {
  engage_speed: number;
  release_speed: number;
  yaw_rate_limit: number;
  slip_angle_limit: number;
};

type LowSpeedSafetyConfig = {
  drift_enabled: boolean;
  stop_speed_epsilon: number;
  normal: LowSpeedRegimeConfig;
  drift: LowSpeedRegimeConfig;
};

export const packagedNativeFactory: NativeDaemonFactory = async (options) =>
  new PackagedNativeDaemon(options);

export default packagedNativeFactory;

class PackagedNativeDaemon implements NativeDaemonHandle {
  private pose = { x: 0, y: 0, yaw: 0 };
  private velocity = { speed: 0, longitudinal: 0, lateral: 0, yaw_rate: 0 };
  private acceleration = { longitudinal: 0, lateral: 0 };
  private steering = { angle: 0, rate: 0 };
  private telemetry = new SimulationTelemetryState();
  private dt = 0;
  private simulationTime = 0;
  private lowSpeedEngaged = false;
  private cumulativeDistance = 0;
  private cumulativeEnergy = 0;

  private vehicleMass = 1200;
  private wheelBase = 2.5;
  private frontAxle = 1.25;
  private rearAxle = 1.25;
  private frontTrack = 1.5;
  private rearTrack = 1.5;
  private wheelRadius = 0.33;
  private brakeSplit = 0.5;
  private steeringMin = -0.6;
  private steeringMax = 0.6;
  private steeringRateMin = -1.0;
  private steeringRateMax = 1.0;
  private maxAccel = 6;
  private maxDecel = -6;
  private maxSpeed = 60;
  private minSpeed = -20;
  private steerResponse = 1.0;

  private lossOfControl: LossOfControlConfig;
  private lowSpeedSafety: LowSpeedSafetyConfig;
  private driftEnabled: boolean;

  constructor(options: {
    model: ModelType;
    vehicleParameters: Record<string, unknown>;
    lowSpeedSafety: LowSpeedSafetyConfig;
    lossOfControl: LossOfControlConfig;
    driftEnabled: boolean;
  }) {
    this.driftEnabled = options.driftEnabled;
    this.lossOfControl = options.lossOfControl;
    this.lowSpeedSafety = options.lowSpeedSafety;
    this.configureFromVehicle(options.model, options.vehicleParameters);
  }

  reset(state: number[], dt: number): void {
    this.pose.x = state[0] ?? 0;
    this.pose.y = state[1] ?? 0;
    this.pose.yaw = state[2] ?? 0;
    this.velocity.speed = state[3] ?? 0;
    this.velocity.longitudinal = this.velocity.speed;
    this.velocity.lateral = 0;
    this.velocity.yaw_rate = 0;
    this.steering.angle = state[4] ?? 0;
    this.steering.rate = 0;
    this.acceleration.longitudinal = 0;
    this.acceleration.lateral = 0;
    this.telemetry = new SimulationTelemetryState();
    this.dt = dt;
    this.simulationTime = 0;
    this.lowSpeedEngaged = false;
    this.cumulativeDistance = 0;
    this.cumulativeEnergy = 0;
  }

  step(control: number[], dt: number): void {
    const [steerRateCommand = 0, accelCommand = 0] = control;
    const clampedRate = clampNumber(
      steerRateCommand * this.steerResponse,
      this.steeringRateMin,
      this.steeringRateMax
    );
    this.steering.rate = clampedRate;
    this.steering.angle = clampNumber(
      this.steering.angle + clampedRate * dt,
      this.steeringMin,
      this.steeringMax
    );

    const accel = clampNumber(accelCommand, this.maxDecel, this.maxAccel);
    const newSpeed = clampNumber(this.velocity.speed + accel * dt, this.minSpeed, this.maxSpeed);
    const meanSpeed = 0.5 * (this.velocity.speed + newSpeed);
    const yawRate = this.computeYawRate(meanSpeed);
    const lateralAccel = yawRate * meanSpeed;
    const lateralVel = yawRate * this.rearAxle;
    const heading = this.pose.yaw + yawRate * dt * 0.5;

    this.pose.x += meanSpeed * Math.cos(heading) * dt;
    this.pose.y += meanSpeed * Math.sin(heading) * dt;
    this.pose.yaw += yawRate * dt;

    this.velocity.speed = newSpeed;
    this.velocity.longitudinal = meanSpeed;
    this.velocity.lateral = lateralVel;
    this.velocity.yaw_rate = yawRate;

    this.acceleration.longitudinal = accel;
    this.acceleration.lateral = lateralAccel;

    this.dt = dt;
    this.simulationTime += dt;
    this.cumulativeDistance += Math.abs(meanSpeed) * dt;
    this.cumulativeEnergy += Math.abs(this.powerFrom(accel, meanSpeed)) * dt;

    this.telemetry = this.buildTelemetry(accelCommand, clampedRate);
  }

  snapshot() {
    return {
      state: [
        this.pose.x,
        this.pose.y,
        this.pose.yaw,
        this.velocity.speed,
        this.steering.angle,
        this.velocity.yaw_rate,
      ],
      telemetry: this.telemetry,
      dt: this.dt,
      simulation_time_s: this.simulationTime,
    };
  }

  speed(): number {
    return this.velocity.speed;
  }

  private configureFromVehicle(model: ModelType, vehicleParameters: Record<string, unknown>): void {
    this.vehicleMass = numberOrFallback(vehicleParameters?.m, this.vehicleMass);
    this.frontAxle = numberOrFallback(vehicleParameters?.a, this.frontAxle);
    this.rearAxle = numberOrFallback(vehicleParameters?.b, this.rearAxle);
    const baseWheelBase = this.frontAxle + this.rearAxle;
    this.wheelBase = Number.isFinite(baseWheelBase) && baseWheelBase > 0 ? baseWheelBase : this.wheelBase;
    this.frontTrack = numberOrFallback(vehicleParameters?.T_f, this.frontTrack);
    this.rearTrack = numberOrFallback(vehicleParameters?.T_r, this.rearTrack);
    this.wheelRadius = numberOrFallback(vehicleParameters?.R_w, this.wheelRadius);
    this.brakeSplit = clampNumber(numberOrFallback(vehicleParameters?.T_sb, this.brakeSplit), 0, 1);

    const steeringParams = ensureObject((vehicleParameters as any).steering);
    this.steeringMin = numberOrFallback(steeringParams.min, this.steeringMin);
    this.steeringMax = numberOrFallback(steeringParams.max, this.steeringMax);
    this.steeringRateMin = numberOrFallback(steeringParams.v_min, this.steeringRateMin);
    this.steeringRateMax = numberOrFallback(steeringParams.v_max, this.steeringRateMax);

    const longParams = ensureObject((vehicleParameters as any).longitudinal);
    const aMax = numberOrFallback(longParams.a_max, this.maxAccel);
    this.maxAccel = Number.isFinite(aMax) ? Math.abs(aMax) : this.maxAccel;
    const vMin = numberOrFallback(longParams.v_min, this.minSpeed);
    this.minSpeed = Number.isFinite(vMin) ? vMin : this.minSpeed;
    const vMax = numberOrFallback(longParams.v_max, this.maxSpeed);
    this.maxSpeed = Number.isFinite(vMax) ? vMax : this.maxSpeed;
    this.maxDecel = -this.maxAccel;

    switch (model) {
      case ModelType.ST:
        this.steerResponse = 0.9;
        break;
      case ModelType.STD:
        this.steerResponse = 0.8;
        break;
      case ModelType.MB:
      default:
        this.steerResponse = 1.0;
        break;
    }
  }

  private computeYawRate(speed: number): number {
    const response = Math.tan(this.steering.angle) * speed;
    return response / Math.max(this.wheelBase, 1e-3);
  }

  private buildTelemetry(accelCommand: number, steerRate: number): SimulationTelemetryState {
    const telemetry = new SimulationTelemetryState();
    telemetry.pose.x = this.pose.x;
    telemetry.pose.y = this.pose.y;
    telemetry.pose.yaw = this.pose.yaw;

    telemetry.velocity.speed = this.velocity.speed;
    telemetry.velocity.longitudinal = this.velocity.longitudinal;
    telemetry.velocity.lateral = this.velocity.lateral;
    telemetry.velocity.yaw_rate = this.velocity.yaw_rate;
    telemetry.velocity.global_x =
      this.velocity.longitudinal * Math.cos(this.pose.yaw) - this.velocity.lateral * Math.sin(this.pose.yaw);
    telemetry.velocity.global_y =
      this.velocity.longitudinal * Math.sin(this.pose.yaw) + this.velocity.lateral * Math.cos(this.pose.yaw);

    telemetry.acceleration.longitudinal = this.acceleration.longitudinal;
    telemetry.acceleration.lateral = this.acceleration.lateral;

    telemetry.steering.actual_rate = steerRate;
    telemetry.steering.desired_rate = steerRate;
    telemetry.steering.actual_angle = this.steering.angle;
    telemetry.steering.desired_angle = this.steering.angle;

    const driveForce = Math.max(0, this.acceleration.longitudinal * this.vehicleMass);
    const brakeForce = Math.max(0, -this.acceleration.longitudinal * this.vehicleMass);
    const rollingForce = 0;
    const dragForce = 0;

    telemetry.controller.acceleration = this.acceleration.longitudinal;
    telemetry.controller.throttle = accelCommand > 0 ? clampNumber(accelCommand / this.maxAccel, 0, 1) : 0;
    telemetry.controller.brake = accelCommand < 0 ? clampNumber(-accelCommand / Math.abs(this.maxDecel), 0, 1) : 0;
    telemetry.controller.drive_force = driveForce;
    telemetry.controller.brake_force = brakeForce;
    telemetry.controller.regen_force = brakeForce;
    telemetry.controller.hydraulic_force = 0;
    telemetry.controller.drag_force = dragForce;
    telemetry.controller.rolling_force = rollingForce;

    const totalDriveTorque = driveForce * this.wheelRadius;
    const totalBrakeTorque = brakeForce * this.wheelRadius;
    telemetry.powertrain.drive_torque = totalDriveTorque;
    telemetry.powertrain.regen_torque = totalBrakeTorque;
    telemetry.powertrain.total_torque = totalDriveTorque - totalBrakeTorque;
    telemetry.powertrain.mechanical_power = this.powerFrom(this.acceleration.longitudinal, this.velocity.speed);
    telemetry.powertrain.battery_power = telemetry.powertrain.mechanical_power;
    telemetry.powertrain.soc = telemetry.powertrain.soc || 0.5;

    const frontTorque = totalDriveTorque * 0.5;
    const rearTorque = totalDriveTorque * 0.5;
    const frontBrake = totalBrakeTorque * this.brakeSplit;
    const rearBrake = totalBrakeTorque * (1 - this.brakeSplit);
    const normalFront = (this.vehicleMass * 9.81 * this.rearAxle) / Math.max(this.wheelBase, 1e-3) * 0.5;
    const normalRear = (this.vehicleMass * 9.81 * this.frontAxle) / Math.max(this.wheelBase, 1e-3) * 0.5;

    telemetry.front_axle.drive_torque = frontTorque;
    telemetry.front_axle.brake_torque = frontBrake;
    telemetry.front_axle.regen_torque = frontBrake;
    telemetry.front_axle.normal_force = normalFront;
    this.populateWheelTelemetry(telemetry.front_axle.left, -1, this.frontTrack, frontTorque * 0.5, frontBrake * 0.5);
    this.populateWheelTelemetry(telemetry.front_axle.right, 1, this.frontTrack, frontTorque * 0.5, frontBrake * 0.5);

    telemetry.rear_axle.drive_torque = rearTorque;
    telemetry.rear_axle.brake_torque = rearBrake;
    telemetry.rear_axle.regen_torque = rearBrake;
    telemetry.rear_axle.normal_force = normalRear;
    this.populateWheelTelemetry(telemetry.rear_axle.left, -1, this.rearTrack, rearTorque * 0.5, rearBrake * 0.5);
    this.populateWheelTelemetry(telemetry.rear_axle.right, 1, this.rearTrack, rearTorque * 0.5, rearBrake * 0.5);

    const slipAngle = Math.atan2(this.velocity.lateral, this.velocity.longitudinal || 1e-6);
    const frontSlip = Math.atan2(
      this.velocity.lateral + this.frontAxle * this.velocity.yaw_rate,
      Math.max(this.velocity.longitudinal, 1e-6)
    );
    const rearSlip = Math.atan2(
      this.velocity.lateral - this.rearAxle * this.velocity.yaw_rate,
      Math.max(this.velocity.longitudinal, 1e-6)
    );
    telemetry.traction.slip_angle = slipAngle;
    telemetry.traction.front_slip_angle = frontSlip;
    telemetry.traction.rear_slip_angle = rearSlip;
    telemetry.traction.lateral_force_saturation = Math.min(1, Math.abs(slipAngle) / 0.5);
    telemetry.traction.drift_mode = this.driftEnabled;

    telemetry.totals.distance_traveled_m = this.cumulativeDistance;
    telemetry.totals.energy_consumed_joules = this.cumulativeEnergy;
    telemetry.totals.simulation_time_s = this.simulationTime;

    const safety = this.evaluateSafety(telemetry);
    telemetry.detector_severity = safety.detector_severity;
    telemetry.safety_stage = safety.safety_stage;
    telemetry.detector_forced = safety.detector_forced;
    telemetry.low_speed_engaged = safety.low_speed_engaged;

    return telemetry;
  }

  private evaluateSafety(telemetry: SimulationTelemetryState): {
    detector_severity: number;
    detector_forced: boolean;
    safety_stage: SafetyStage;
    low_speed_engaged: boolean;
  } {
    const slipAngle = Math.abs(telemetry.traction.slip_angle);
    const yawRate = Math.abs(telemetry.velocity.yaw_rate);
    const latAccel = Math.abs(telemetry.acceleration.lateral);
    const speed = Math.abs(telemetry.velocity.speed);
    const slipRatio = Math.max(
      Math.abs(telemetry.front_axle.left.slip_ratio),
      Math.abs(telemetry.front_axle.right.slip_ratio),
      Math.abs(telemetry.rear_axle.left.slip_ratio),
      Math.abs(telemetry.rear_axle.right.slip_ratio)
    );

    const normalRegime = this.driftEnabled ? this.lowSpeedSafety.drift : this.lowSpeedSafety.normal;
    const lowSpeedEngaged =
      speed < (normalRegime.engage_speed ?? 0.4) &&
      Math.abs(telemetry.velocity.yaw_rate) < (normalRegime.yaw_rate_limit ?? 0.5) &&
      Math.abs(telemetry.traction.slip_angle) < (normalRegime.slip_angle_limit ?? 0.35);

    const releaseConditions =
      speed > (normalRegime.release_speed ?? 0.8) ||
      Math.abs(telemetry.velocity.yaw_rate) > (normalRegime.yaw_rate_limit ?? 0.5) * 1.2 ||
      Math.abs(telemetry.traction.slip_angle) > (normalRegime.slip_angle_limit ?? 0.35) * 1.2;

    if (this.lowSpeedEngaged && releaseConditions) {
      this.lowSpeedEngaged = false;
    } else if (!this.lowSpeedEngaged && lowSpeedEngaged) {
      this.lowSpeedEngaged = true;
    }

    const loc = this.lossOfControl;
    const severityYaw = severityFromChannel(yawRate, loc.yaw_rate);
    const severitySlip = severityFromChannel(slipAngle, loc.slip_angle);
    const severityLatAccel = severityFromChannel(latAccel, loc.lateral_accel);
    const severitySlipRatio = severityFromChannel(slipRatio, loc.slip_ratio);
    const severity = Math.max(severityYaw, severitySlip, severityLatAccel, severitySlipRatio);

    let stage = SafetyStage.Normal;
    if (this.lowSpeedEngaged) {
      stage = SafetyStage.LowSpeed;
    } else if (severity > 0.8) {
      stage = SafetyStage.Oversteer;
    } else if (severity > 0.6) {
      stage = SafetyStage.Understeer;
    }

    return {
      detector_severity: severity,
      detector_forced: false,
      safety_stage: stage,
      low_speed_engaged: this.lowSpeedEngaged,
    };
  }

  private populateWheelTelemetry(
    wheel: { speed: number; slip_ratio: number; friction_utilization: number },
    sideSign: number,
    track: number,
    driveTorque: number,
    brakeTorque: number
  ) {
    const speed = this.velocity.longitudinal + sideSign * this.velocity.yaw_rate * (track * 0.5);
    const driveForce = driveTorque / Math.max(this.wheelRadius, 1e-3);
    const brakeForce = brakeTorque / Math.max(this.wheelRadius, 1e-3);
    const slipRatio = speed !== 0 ? (driveForce - brakeForce) / Math.abs(speed) : 0;

    wheel.speed = speed;
    wheel.slip_ratio = slipRatio;
    wheel.friction_utilization = clampNumber(Math.abs(slipRatio), 0, 1);
  }

  private powerFrom(accel: number, speed: number): number {
    return accel * this.vehicleMass * speed;
  }
}

function severityFromChannel(value: number, config?: LossOfControlChannelThresholds): number {
  if (!config) return 0;
  if (value <= config.threshold) return 0;
  return Math.min(1, (value - config.threshold) * (config.rate ?? 1));
}

function ensureObject(value: unknown): Record<string, unknown> {
  if (value && typeof value === 'object') {
    return value as Record<string, unknown>;
  }
  return {};
}

function numberOrFallback(value: unknown, fallback: number): number {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : fallback;
}

function clampNumber(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max);
}
