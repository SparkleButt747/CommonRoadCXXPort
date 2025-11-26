import { SimulationTelemetryState, SafetyStage } from '../telemetry/index.js';
import type { BackendSnapshot, SimulationBackend } from './backend.js';

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

  status(state: number[], speed: number): { severity: number; stage: SafetyStage; detectorForced: boolean } {
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
    return { severity, stage, detectorForced };
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
  private safety: LowSpeedSafety;
  private loss: LossOfControlDetector;
  ready: Promise<void> = Promise.resolve();

  constructor(private readonly options: JsBackendOptions) {
    this.safety = new LowSpeedSafety(options.lowSpeed, options.driftEnabled);
    this.loss = new LossOfControlDetector(options.lossConfig);
  }

  reset(initial: number[], dt: number): void {
    this.dt = dt;
    this.simTime = 0;
    this.safety.reset();
    this.loss.reset();
    this.state = this.initializeState(initial);
    this.telemetry = new SimulationTelemetryState();
    this.updateTelemetry(0, 0, 0);
  }

  step(control: number[], dt: number): void {
    const steerRateCmd = control[0] ?? 0;
    const accelCmd = control[1] ?? 0;
    const steering = this.options.params.steering ?? { max: 0.5, min: -0.5, v_max: 0.5, v_min: -0.5 };
    const longitudinal = this.options.params.longitudinal ?? { a_max: 5, v_max: 50, v_min: -5 };
    const wheelBase = (this.options.params.a ?? 1.0) + (this.options.params.b ?? 1.0);
    const rearLength = this.options.params.b ?? wheelBase * 0.5;
    const mass = this.options.params.m ?? 1500;

    const steerRate = Math.max(steering.v_min, Math.min(steering.v_max, steerRateCmd));
    const accel = Math.max(-longitudinal.a_max, Math.min(longitudinal.a_max, accelCmd));

    const dtClamped = dt ?? this.dt;

    // State layout: [x, y, yaw, v_long, v_lat, yaw_rate, slip_angle, steering, front_speed, rear_speed]
    const yaw = this.state[2];
    let steeringAngle = this.state[7];
    steeringAngle += steerRate * dtClamped;
    steeringAngle = Math.max(steering.min, Math.min(steering.max, steeringAngle));

    let vLong = this.state[3] + accel * dtClamped;
    vLong = Math.max(longitudinal.v_min ?? -50, Math.min(longitudinal.v_max ?? 50, vLong));

    const beta = Math.atan((rearLength / Math.max(wheelBase, 1e-6)) * Math.tan(steeringAngle));
    const speed = Math.max(0, vLong);
    const yawRate = (speed * Math.sin(beta)) / Math.max(rearLength, 1e-6);
    const vLat = speed * Math.sin(beta);

    const vx = speed * Math.cos(yaw + beta);
    const vy = speed * Math.sin(yaw + beta);

    this.state[0] += vx * dtClamped;
    this.state[1] += vy * dtClamped;
    this.state[2] += yawRate * dtClamped;
    this.state[3] = vLong;
    this.state[4] = vLat;
    this.state[5] = yawRate;
    this.state[6] = beta;
    this.state[7] = steeringAngle;
    this.state[8] = Math.max(0, speed + (this.options.params.a ?? 0) * yawRate);
    this.state[9] = Math.max(0, speed - (this.options.params.b ?? 0) * yawRate);

    const lateralAccel = yawRate * speed;

    this.safety.setDriftEnabled(this.options.driftEnabled);
    this.safety.apply(this.state, speed, 5, 6, 4, 7, wheelBase, rearLength);
    const severity = this.loss.update(dtClamped, this.state[5], this.state[6], lateralAccel, this.computeSlipRatios(speed));

    this.simTime += dtClamped;
    this.updateTelemetry(accel, steerRate, severity);
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

  private computeSlipRatios(speed: number): number[] {
    const denom = Math.max(speed, 1e-6);
    const front = this.state[8] ?? speed;
    const rear = this.state[9] ?? speed;
    return [(front - speed) / denom, (rear - speed) / denom];
  }

  private updateTelemetry(accel: number, steerRate: number, detectorSeverity: number): void {
    const wheelBase = (this.options.params.a ?? 1.0) + (this.options.params.b ?? 1.0);
    const mass = this.options.params.m ?? 1500;
    const wheelRadius = this.options.params.R_w ?? 0.3;
    const frontSplit = this.options.params.T_sb ?? 0.5;
    const rearSplit = 1 - frontSplit;
    const normalForce = (mass * 9.81) / 2;

    const speed = Math.max(0, this.state[3]);
    const slipAngle = this.state[6] ?? 0;
    const yawRate = this.state[5] ?? 0;
    const vLat = this.state[4] ?? 0;
    const vx = speed * Math.cos(this.state[2] + slipAngle);
    const vy = speed * Math.sin(this.state[2] + slipAngle);
    const lateralAccel = yawRate * speed;

    const steeringAngle = this.state[7] ?? 0;
    const slipRatios = this.computeSlipRatios(speed);

    const driveForce = accel >= 0 ? accel * mass : 0;
    const brakeForce = accel < 0 ? -accel * mass : 0;
    const wheelTorque = driveForce * wheelRadius;
    const brakeTorque = brakeForce * wheelRadius;

    const telem = new SimulationTelemetryState();
    telem.pose.x = this.state[0];
    telem.pose.y = this.state[1];
    telem.pose.yaw = this.state[2];

    telem.velocity.speed = speed;
    telem.velocity.longitudinal = speed;
    telem.velocity.lateral = vLat;
    telem.velocity.yaw_rate = yawRate;
    telem.velocity.global_x = vx;
    telem.velocity.global_y = vy;

    telem.acceleration.longitudinal = accel;
    telem.acceleration.lateral = lateralAccel;

    telem.traction.slip_angle = slipAngle;
    telem.traction.front_slip_angle = slipAngle;
    telem.traction.rear_slip_angle = slipAngle;
    telem.traction.lateral_force_saturation = Math.min(1, Math.abs(lateralAccel) / Math.max(1, speed));
    telem.traction.drift_mode = this.options.driftEnabled;

    telem.steering.desired_angle = steeringAngle;
    telem.steering.actual_angle = steeringAngle;
    telem.steering.desired_rate = steerRate;
    telem.steering.actual_rate = steerRate;

    telem.controller.acceleration = accel;
    telem.controller.throttle = accel > 0 ? accel / Math.max(this.options.params.longitudinal?.a_max ?? 1, 1e-6) : 0;
    telem.controller.brake = accel < 0 ? -accel / Math.max(this.options.params.longitudinal?.a_max ?? 1, 1e-6) : 0;
    telem.controller.drive_force = driveForce;
    telem.controller.brake_force = brakeForce;
    telem.controller.regen_force = 0;
    telem.controller.hydraulic_force = brakeForce;
    telem.controller.drag_force = 0.5 * driveForce * 0;
    telem.controller.rolling_force = 0;

    telem.powertrain.drive_torque = wheelTorque;
    telem.powertrain.regen_torque = 0;
    telem.powertrain.total_torque = wheelTorque;
    telem.powertrain.mechanical_power = wheelTorque * speed / Math.max(wheelRadius, 1e-6);
    telem.powertrain.battery_power = telem.powertrain.mechanical_power;
    telem.powertrain.soc = 0.5;

    telem.front_axle.drive_torque = wheelTorque * frontSplit;
    telem.rear_axle.drive_torque = wheelTorque * rearSplit;
    telem.front_axle.brake_torque = brakeTorque * frontSplit;
    telem.rear_axle.brake_torque = brakeTorque * rearSplit;
    telem.front_axle.regen_torque = 0;
    telem.rear_axle.regen_torque = 0;
    telem.front_axle.normal_force = normalForce;
    telem.rear_axle.normal_force = normalForce;

    telem.front_axle.left.speed = speed;
    telem.front_axle.right.speed = speed;
    telem.rear_axle.left.speed = speed;
    telem.rear_axle.right.speed = speed;

    telem.front_axle.left.slip_ratio = slipRatios[0];
    telem.front_axle.right.slip_ratio = slipRatios[0];
    telem.rear_axle.left.slip_ratio = slipRatios[1];
    telem.rear_axle.right.slip_ratio = slipRatios[1];

    telem.front_axle.left.friction_utilization = Math.abs(slipRatios[0]);
    telem.front_axle.right.friction_utilization = Math.abs(slipRatios[0]);
    telem.rear_axle.left.friction_utilization = Math.abs(slipRatios[1]);
    telem.rear_axle.right.friction_utilization = Math.abs(slipRatios[1]);

    const safetyStatus = this.safety.status(this.state, speed);
    telem.detector_severity = Math.max(detectorSeverity, safetyStatus.severity);
    telem.safety_stage = safetyStatus.stage;
    telem.detector_forced = safetyStatus.detectorForced;
    telem.low_speed_engaged = safetyStatus.stage !== SafetyStage.Normal;

    telem.totals.distance_traveled_m = this.telemetry.totals.distance_traveled_m + speed * this.dt;
    telem.totals.energy_consumed_joules = this.telemetry.totals.energy_consumed_joules + driveForce * speed * this.dt;
    telem.totals.simulation_time_s = this.simTime;

    this.telemetry = telem;
  }
}

