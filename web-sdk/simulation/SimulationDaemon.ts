import {
  AccelerationTelemetryState,
  ControllerTelemetryState,
  DerivedTelemetryState,
  PoseTelemetryState,
  SafetyStage,
  SimulationTelemetry,
  SimulationTelemetryState,
  SteeringTelemetryState,
  TractionTelemetryState,
  VelocityTelemetryState,
} from '../telemetry/index.js';
import { ConfigManager } from '../io/ConfigManager.js';
import { ControlMode, ModelTimingInfo, ModelType } from './types.js';
export { ControlMode, ModelTimingInfo, ModelType } from './types.js';

export interface DriverIntent {
  throttle: number;
  brake: number;
}

export interface UserInput {
  control_mode?: ControlMode;
  longitudinal: DriverIntent;
  steering_nudge?: number;
  steering_angle?: number;
  axle_torques?: number[];
  drift_toggle?: number;
  timestamp: number;
  dt: number;
}

export interface UserInputLimitsConfig {
  min_throttle?: number;
  max_throttle?: number;
  min_brake?: number;
  max_brake?: number;
  min_steering_nudge?: number;
  max_steering_nudge?: number;
  min_drift_toggle?: number;
  max_drift_toggle?: number;
  min_steering_angle?: number;
  max_steering_angle?: number;
  min_axle_torque?: number[];
  max_axle_torque?: number[];
}

export class UserInputLimits {
  min_throttle = 0.0;
  max_throttle = 1.0;
  min_brake = 0.0;
  max_brake = 1.0;
  min_steering_nudge = -1.0;
  max_steering_nudge = 1.0;
  min_drift_toggle = 0.0;
  max_drift_toggle = 1.0;
  min_steering_angle = 0.0;
  max_steering_angle = 0.0;
  min_axle_torque: number[] = [];
  max_axle_torque: number[] = [];

  constructor(overrides: UserInputLimitsConfig = {}) {
    Object.assign(this, overrides);
  }

  clamp(input: UserInput): UserInput {
    this.validate(input);
    const copy: UserInput = { ...input, longitudinal: { ...input.longitudinal } };
    const mode = copy.control_mode ?? ControlMode.Keyboard;
    if (mode === ControlMode.Keyboard) {
      copy.longitudinal.throttle = clamp(copy.longitudinal.throttle, this.min_throttle, this.max_throttle);
      copy.longitudinal.brake = clamp(copy.longitudinal.brake, this.min_brake, this.max_brake);
      copy.steering_nudge = clamp(copy.steering_nudge ?? 0.0, this.min_steering_nudge, this.max_steering_nudge);
    } else {
      copy.steering_angle = clamp(copy.steering_angle ?? 0.0, this.min_steering_angle, this.max_steering_angle);
      if (Array.isArray(copy.axle_torques) &&
          copy.axle_torques.length === this.min_axle_torque.length &&
          this.min_axle_torque.length === this.max_axle_torque.length) {
        copy.axle_torques = copy.axle_torques.map((torque, idx) =>
          clamp(torque, this.min_axle_torque[idx], this.max_axle_torque[idx])
        );
      }
    }
    if (copy.drift_toggle !== undefined) {
      copy.drift_toggle = clamp(copy.drift_toggle, this.min_drift_toggle, this.max_drift_toggle);
    }
    return copy;
  }

  validate(input: UserInput): void {
    const mode = input.control_mode ?? ControlMode.Keyboard;
    requireFinite(input.timestamp, 'timestamp');
    if (input.timestamp < 0) {
      throw new Error(`UserInput.timestamp must be non-negative; got ${input.timestamp}`);
    }
    requireFinite(input.dt, 'dt');
    if (input.dt <= 0) {
      throw new Error(`UserInput.dt must be positive; got ${input.dt}`);
    }

    if (mode === ControlMode.Keyboard) {
      requireFinite(input.longitudinal.throttle, 'longitudinal.throttle');
      requireInRange(input.longitudinal.throttle, 'longitudinal.throttle', this.min_throttle, this.max_throttle);
      requireFinite(input.longitudinal.brake, 'longitudinal.brake');
      requireInRange(input.longitudinal.brake, 'longitudinal.brake', this.min_brake, this.max_brake);
      const steeringNudge = input.steering_nudge ?? 0.0;
      requireFinite(steeringNudge, 'steering_nudge');
      requireInRange(steeringNudge, 'steering_nudge', this.min_steering_nudge, this.max_steering_nudge);
    } else if (mode === ControlMode.Direct) {
      const steeringAngle = input.steering_angle ?? 0.0;
      requireFinite(steeringAngle, 'steering_angle');
      requireInRange(steeringAngle, 'steering_angle', this.min_steering_angle, this.max_steering_angle);
      const enforceTorque =
        (this.min_axle_torque.length > 0 || this.max_axle_torque.length > 0 || (input.axle_torques?.length ?? 0) > 0);
      if (enforceTorque) {
        if (this.min_axle_torque.length !== this.max_axle_torque.length) {
          throw new Error('UserInputLimits torque bounds must be sized consistently');
        }
        if ((input.axle_torques?.length ?? 0) !== this.min_axle_torque.length) {
          throw new Error(
            `UserInput.axle_torques length ${(input.axle_torques?.length ?? 0)} does not match driven axle count ${this.min_axle_torque.length}`
          );
        }
        input.axle_torques?.forEach((torque, idx) => {
          requireFinite(torque, 'axle_torques');
          requireInRange(torque, 'axle_torques', this.min_axle_torque[idx], this.max_axle_torque[idx]);
        });
      }
    } else {
      throw new Error('Unknown UserInput control mode');
    }

    if (input.drift_toggle !== undefined) {
      requireFinite(input.drift_toggle, 'drift_toggle');
      requireInRange(input.drift_toggle, 'drift_toggle', this.min_drift_toggle, this.max_drift_toggle);
    }
  }
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max);
}

function requireFinite(value: number, field: string): void {
  if (!Number.isFinite(value)) {
    throw new Error(`UserInput.${field} must be finite; got ${value}`);
  }
}

function requireInRange(value: number, field: string, min: number, max: number): void {
  if (value < min || value > max) {
    throw new Error(`UserInput.${field} of ${value} outside [${min}, ${max}]`);
  }
}

export interface InitParams {
  model?: ModelType;
  vehicle_id?: number;
  config_root?: string;
  parameter_root?: string;
  drift_enabled?: boolean;
  control_mode?: ControlMode;
  backend?: SimulationBackend;
  limits?: UserInputLimits;
  timing?: ModelTimingInfo;
}

export interface ResetParams {
  model?: ModelType;
  vehicle_id?: number;
  initial_state?: number[];
  dt?: number;
  drift_enabled?: boolean;
  control_mode?: ControlMode;
}

export interface SimulationSnapshot {
  state: number[];
  telemetry: SimulationTelemetry;
  dt: number;
  simulation_time_s: number;
}

export interface SimulationBackend {
  reset(state: number[], dt: number): void;
  step(control: number[], dt: number): void;
  snapshot(): { state: number[] };
  speed(): number;
}

export interface StepSchedule {
  requested_dt: number;
  clamped_dt: number;
  substeps: number[];
  clamped_to_min: boolean;
  used_substeps: boolean;
}

const kMinStableDt = 0.001;
const kDefaultTimings: Record<ModelType, ModelTimingInfo> = {
  [ModelType.MB]: { nominal_dt: 0.005, max_dt: 0.005 },
  [ModelType.ST]: { nominal_dt: 0.01, max_dt: 0.02 },
  [ModelType.STD]: { nominal_dt: 0.01, max_dt: 0.01 },
};

class ModelTiming {
  constructor(private info: ModelTimingInfo) {}

  planSteps(requested_dt: number): StepSchedule {
    if (!Number.isFinite(requested_dt)) {
      throw new Error('Requested dt must be finite');
    }
    if (requested_dt <= 0) {
      throw new Error(`Requested dt must be positive; got ${requested_dt}`);
    }
    if (!(this.info.max_dt > 0)) {
      throw new Error('ModelTiming requires a positive max_dt configuration');
    }

    const total_dt = Math.max(requested_dt, kMinStableDt);
    const schedule: StepSchedule = {
      requested_dt,
      clamped_dt: total_dt,
      substeps: [],
      clamped_to_min: total_dt > requested_dt,
      used_substeps: false,
    };

    let steps = Math.ceil(total_dt / this.info.max_dt);
    steps = Math.max(1, steps);
    while (steps > 1 && total_dt / steps < kMinStableDt) {
      steps -= 1;
    }

    const base_dt = total_dt / steps;
    let accumulated = 0;
    for (let i = 0; i < steps; i += 1) {
      const last = i === steps - 1;
      const dt = last ? total_dt - accumulated : base_dt;
      accumulated += dt;
      schedule.substeps.push(dt);
    }
    schedule.used_substeps = schedule.substeps.length > 1;
    return schedule;
  }
}

class MockBackend implements SimulationBackend {
  private pose = new PoseTelemetryState();
  private velocity = new VelocityTelemetryState();
  private yaw_rate = 0;
  private dt = 0;

  reset(state: number[], dt: number): void {
    this.pose.x = state[0] ?? 0;
    this.pose.y = state[1] ?? 0;
    this.pose.yaw = state[2] ?? 0;
    this.velocity.speed = 0;
    this.velocity.longitudinal = 0;
    this.velocity.lateral = 0;
    this.velocity.yaw_rate = 0;
    this.yaw_rate = 0;
    this.dt = dt;
  }

  step(control: number[], dt: number): void {
    const [steer_rate, accel] = control;
    this.yaw_rate = steer_rate ?? this.yaw_rate;
    this.velocity.speed = Math.max(0, this.velocity.speed + (accel ?? 0) * dt);
    const heading = this.pose.yaw;
    const mean_speed = this.velocity.speed;
    this.pose.x += mean_speed * Math.cos(heading) * dt;
    this.pose.y += mean_speed * Math.sin(heading) * dt;
    this.pose.yaw += this.yaw_rate * dt;
    this.velocity.longitudinal = mean_speed;
    this.velocity.lateral = 0;
    this.velocity.yaw_rate = this.yaw_rate;
  }

  snapshot(): { state: number[] } {
    return { state: [this.pose.x, this.pose.y, this.pose.yaw, this.velocity.longitudinal] };
  }

  speed(): number {
    return this.velocity.speed;
  }
}

export class SimulationDaemon {
  private model: ModelType;
  private vehicleId: number;
  private driftEnabled: boolean;
  private controlMode: ControlMode;
  private backend: SimulationBackend;
  private limits: UserInputLimits;
  private timing: ModelTiming;
  private configManager: ConfigManager;
  private lastTelemetry: SimulationTelemetryState = new SimulationTelemetryState();
  private cumulativeDistance = 0;
  private cumulativeEnergy = 0;
  private simulationTime = 0;
  private lastDt: number;

  constructor(private readonly init: InitParams = {}) {
    this.model = init.model ?? ModelType.MB;
    this.vehicleId = init.vehicle_id ?? 1;
    this.driftEnabled = init.drift_enabled ?? false;
    this.controlMode = init.control_mode ?? ControlMode.Keyboard;
    this.backend = init.backend ?? new MockBackend();
    this.limits = init.limits ?? new UserInputLimits();
    const timingInfo = init.timing ?? kDefaultTimings[this.model];
    this.timing = new ModelTiming(timingInfo);
    this.configManager = new ConfigManager(init.config_root, init.parameter_root);
    const defaultDt = timingInfo.nominal_dt;
    this.lastDt = defaultDt;
    this.backend.reset(init.initial_state ?? [], defaultDt);
  }

  reset(params: ResetParams = {}): void {
    if (params.model) {
      this.model = params.model;
    }
    if (params.vehicle_id) {
      this.vehicleId = params.vehicle_id;
    }
    if (params.control_mode) {
      this.controlMode = params.control_mode;
    }
    if (params.drift_enabled !== undefined) {
      this.driftEnabled = params.drift_enabled;
    }
    const timingInfo = this.init.timing ?? kDefaultTimings[this.model];
    this.timing = new ModelTiming(timingInfo);
    const schedule = this.timing.planSteps(params.dt ?? timingInfo.nominal_dt);
    this.lastDt = schedule.clamped_dt;
    this.backend.reset(params.initial_state ?? [], schedule.substeps[0]);
    this.cumulativeDistance = 0;
    this.cumulativeEnergy = 0;
    this.simulationTime = 0;
    this.lastTelemetry = new SimulationTelemetryState();
  }

  setDriftEnabled(enabled: boolean): void {
    this.driftEnabled = enabled;
  }

  telemetry(): SimulationTelemetry {
    return this.lastTelemetry;
  }

  snapshot(): SimulationSnapshot {
    return {
      state: this.backend.snapshot().state,
      telemetry: this.lastTelemetry,
      dt: this.lastDt,
      simulation_time_s: this.simulationTime,
    };
  }

  step(input: UserInput): SimulationTelemetry {
    const working: UserInput = { ...input, control_mode: this.controlMode };
    const sanitized = this.limits.clamp(working);
    if (sanitized.drift_toggle !== undefined) {
      this.setDriftEnabled(sanitized.drift_toggle >= 0.5);
    }

    const schedule = this.timing.planSteps(sanitized.dt);
    this.lastDt = schedule.clamped_dt;

    let accelCommand = 0;
    let steerRate = 0;
    let steerAngle = sanitized.steering_angle ?? 0;
    if (sanitized.control_mode === ControlMode.Keyboard) {
      accelCommand = sanitized.longitudinal.throttle - sanitized.longitudinal.brake;
      const nudge = sanitized.steering_nudge ?? 0;
      steerRate = nudge;
    } else {
      accelCommand = (sanitized.axle_torques ?? []).reduce((sum, torque) => sum + torque, 0);
      steerRate = 0;
    }

    for (const dt of schedule.substeps) {
      const control = [steerRate, accelCommand];
      this.backend.step(control, dt);
      this.cumulativeDistance += Math.abs(this.backend.speed()) * dt;
      this.cumulativeEnergy += accelCommand * this.backend.speed() * dt;
      this.simulationTime += dt;
      steerAngle += steerRate * dt;
    }

    this.lastTelemetry = this.buildTelemetry(accelCommand, steerRate, steerAngle);
    return this.lastTelemetry;
  }

  stepBatch(inputs: UserInput[]): SimulationTelemetry[] {
    return inputs.map((entry) => this.step(entry));
  }

  private buildTelemetry(accel: number, steerRate: number, steerAngle: number): SimulationTelemetryState {
    const telemetry = new SimulationTelemetryState();
    const snapshot = this.backend.snapshot();
    const [x, y, yaw, speed = this.backend.speed()] = snapshot.state;
    telemetry.pose.x = x ?? 0;
    telemetry.pose.y = y ?? 0;
    telemetry.pose.yaw = yaw ?? 0;
    telemetry.velocity.speed = speed ?? 0;
    telemetry.velocity.longitudinal = speed ?? 0;
    telemetry.velocity.yaw_rate = steerRate;
    telemetry.acceleration.longitudinal = accel;
    telemetry.steering.desired_rate = steerRate;
    telemetry.steering.actual_rate = steerRate;
    telemetry.steering.actual_angle = steerAngle;
    telemetry.steering.desired_angle = steerAngle;
    telemetry.controller.acceleration = accel;
    telemetry.controller.throttle = Math.max(0, accel);
    telemetry.controller.brake = Math.max(0, -accel);
    telemetry.powertrain.drive_torque = Math.max(0, accel);
    telemetry.powertrain.regen_torque = Math.max(0, -accel);
    telemetry.powertrain.total_torque = telemetry.powertrain.drive_torque - telemetry.powertrain.regen_torque;
    telemetry.powertrain.mechanical_power = accel * speed;
    telemetry.powertrain.battery_power = telemetry.powertrain.mechanical_power;
    telemetry.totals.distance_traveled_m = this.cumulativeDistance;
    telemetry.totals.energy_consumed_joules = this.cumulativeEnergy;
    telemetry.totals.simulation_time_s = this.simulationTime;
    telemetry.traction.drift_mode = this.driftEnabled;
    telemetry.safety_stage = SafetyStage.Normal;
    return telemetry;
  }
}
