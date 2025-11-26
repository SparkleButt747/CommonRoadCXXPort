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
import { BackendSnapshot, HybridSimulationBackend, NativeDaemonFactory, SimulationBackend } from './backend.js';
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
  native_factory?: NativeDaemonFactory;
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

function mergeTelemetry(base: SimulationTelemetry | undefined): SimulationTelemetryState {
  const telemetry = new SimulationTelemetryState();
  if (!base) return telemetry;
  Object.assign(telemetry.pose, base.pose ?? {});
  Object.assign(telemetry.velocity, base.velocity ?? {});
  Object.assign(telemetry.acceleration, base.acceleration ?? {});
  Object.assign(telemetry.traction, base.traction ?? {});
  Object.assign(telemetry.steering, base.steering ?? {});
  Object.assign(telemetry.controller, base.controller ?? {});
  Object.assign(telemetry.powertrain, base.powertrain ?? {});
  Object.assign(telemetry.front_axle, base.front_axle ?? {});
  if (base.front_axle?.left) Object.assign(telemetry.front_axle.left, base.front_axle.left);
  if (base.front_axle?.right) Object.assign(telemetry.front_axle.right, base.front_axle.right);
  Object.assign(telemetry.rear_axle, base.rear_axle ?? {});
  if (base.rear_axle?.left) Object.assign(telemetry.rear_axle.left, base.rear_axle.left);
  if (base.rear_axle?.right) Object.assign(telemetry.rear_axle.right, base.rear_axle.right);
  Object.assign(telemetry.totals, base.totals ?? {});
  telemetry.detector_severity = base.detector_severity ?? telemetry.detector_severity;
  telemetry.safety_stage = base.safety_stage ?? telemetry.safety_stage;
  telemetry.detector_forced = base.detector_forced ?? telemetry.detector_forced;
  telemetry.low_speed_engaged = base.low_speed_engaged ?? telemetry.low_speed_engaged;
  return telemetry;
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
  ready: Promise<void>;

  constructor(private readonly init: InitParams = {}) {
    this.model = init.model ?? ModelType.MB;
    this.vehicleId = init.vehicle_id ?? 1;
    this.driftEnabled = init.drift_enabled ?? false;
    this.controlMode = init.control_mode ?? ControlMode.Keyboard;
    this.configManager = new ConfigManager(init.config_root, init.parameter_root);
    this.backend = init.backend ?? new HybridSimulationBackend({
      model: this.model,
      vehicleId: this.vehicleId,
      configManager: this.configManager,
      nativeFactory: init.native_factory,
      driftEnabled: this.driftEnabled,
    });
    this.limits = init.limits ?? new UserInputLimits();
    const timingInfo = init.timing ?? kDefaultTimings[this.model];
    this.timing = new ModelTiming(timingInfo);
    this.lastDt = timingInfo.nominal_dt;
    this.ready = this.performReset({
      model: this.model,
      vehicle_id: this.vehicleId,
      control_mode: this.controlMode,
      drift_enabled: this.driftEnabled,
      initial_state: init.initial_state ?? [],
      dt: timingInfo.nominal_dt,
    });
  }

  reset(params: ResetParams = {}): Promise<void> {
    this.ready = this.performReset(params);
    return this.ready;
  }

  setDriftEnabled(enabled: boolean): void {
    this.driftEnabled = enabled;
  }

  telemetry(): SimulationTelemetry {
    return this.lastTelemetry;
  }

  async snapshot(): Promise<SimulationSnapshot> {
    await this.ready;
    const snap = this.backend.snapshot();
    return {
      state: snap.state,
      telemetry: snap.telemetry ?? this.lastTelemetry,
      dt: snap.dt ?? this.lastDt,
      simulation_time_s: snap.simulation_time_s ?? this.simulationTime,
    };
  }

  async step(input: UserInput): Promise<SimulationTelemetry> {
    await this.ready;
    const working: UserInput = { ...input, control_mode: this.controlMode };
    const sanitized = this.limits.clamp(working);
    if (sanitized.drift_toggle !== undefined) {
      this.setDriftEnabled(sanitized.drift_toggle >= 0.5);
    }

    const schedule = this.timing.planSteps(sanitized.dt);
    this.lastDt = schedule.clamped_dt;

    let accelCommand = 0;
    let steerRate = 0;
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
      await this.backend.step(control, dt);
      const speed = this.backend.speed();
      this.cumulativeDistance += Math.abs(speed) * dt;
      this.cumulativeEnergy += accelCommand * speed * dt;
      this.simulationTime += dt;
    }

    this.lastTelemetry = this.buildTelemetry(this.backend.snapshot());
    this.cumulativeDistance = this.lastTelemetry.totals.distance_traveled_m ?? this.cumulativeDistance;
    this.cumulativeEnergy = this.lastTelemetry.totals.energy_consumed_joules ?? this.cumulativeEnergy;
    this.simulationTime = this.lastTelemetry.totals.simulation_time_s ?? this.simulationTime;
    return this.lastTelemetry;
  }

  async stepBatch(inputs: UserInput[]): Promise<SimulationTelemetry[]> {
    const outputs: SimulationTelemetry[] = [];
    for (const entry of inputs) {
      outputs.push(await this.step(entry));
    }
    return outputs;
  }

  private buildTelemetry(snapshot: BackendSnapshot): SimulationTelemetryState {
    if (snapshot.telemetry) {
      const telemetry = mergeTelemetry(snapshot.telemetry);
      this.cumulativeDistance = telemetry.totals.distance_traveled_m ?? this.cumulativeDistance;
      this.cumulativeEnergy = telemetry.totals.energy_consumed_joules ?? this.cumulativeEnergy;
      this.simulationTime = snapshot.simulation_time_s ?? telemetry.totals.simulation_time_s ?? this.simulationTime;
      return telemetry;
    }

    const telemetry = mergeTelemetry(undefined);
    const [x, y, yaw, speed = this.backend.speed()] = snapshot.state;
    telemetry.pose.x = x ?? telemetry.pose.x;
    telemetry.pose.y = y ?? telemetry.pose.y;
    telemetry.pose.yaw = yaw ?? telemetry.pose.yaw;
    telemetry.velocity.speed = speed ?? telemetry.velocity.speed;
    telemetry.velocity.longitudinal = telemetry.velocity.longitudinal ?? telemetry.velocity.speed;
    telemetry.velocity.lateral = telemetry.velocity.lateral ?? 0;
    telemetry.velocity.yaw_rate = telemetry.velocity.yaw_rate ?? (snapshot.state[5] ?? 0);
    telemetry.velocity.global_x = telemetry.velocity.global_x ?? telemetry.velocity.longitudinal * Math.cos(telemetry.pose.yaw);
    telemetry.velocity.global_y = telemetry.velocity.global_y ?? telemetry.velocity.longitudinal * Math.sin(telemetry.pose.yaw);

    telemetry.acceleration.longitudinal = telemetry.acceleration.longitudinal ?? 0;
    telemetry.acceleration.lateral = telemetry.acceleration.lateral ?? 0;

    const slip = snapshot.state[6] ?? telemetry.traction.slip_angle ?? 0;
    telemetry.traction.slip_angle = slip;
    telemetry.traction.front_slip_angle = telemetry.traction.front_slip_angle ?? slip;
    telemetry.traction.rear_slip_angle = telemetry.traction.rear_slip_angle ?? slip;
    const lateralForceSat = telemetry.traction.lateral_force_saturation ?? Math.min(1, Math.abs(telemetry.acceleration.lateral) / Math.max(telemetry.velocity.speed, 1e-6));
    telemetry.traction.lateral_force_saturation = lateralForceSat;
    telemetry.traction.drift_mode = this.driftEnabled;

    telemetry.steering.actual_angle = snapshot.state[7] ?? telemetry.steering.actual_angle;
    telemetry.steering.desired_angle = telemetry.steering.desired_angle ?? telemetry.steering.actual_angle;
    telemetry.steering.actual_rate = telemetry.steering.actual_rate ?? 0;
    telemetry.steering.desired_rate = telemetry.steering.desired_rate ?? 0;

    telemetry.controller.acceleration = telemetry.controller.acceleration ?? telemetry.acceleration.longitudinal ?? 0;
    telemetry.controller.throttle = telemetry.controller.throttle ?? 0;
    telemetry.controller.brake = telemetry.controller.brake ?? 0;
    telemetry.controller.drive_force = telemetry.controller.drive_force ?? 0;
    telemetry.controller.brake_force = telemetry.controller.brake_force ?? 0;
    telemetry.controller.regen_force = telemetry.controller.regen_force ?? 0;
    telemetry.controller.hydraulic_force = telemetry.controller.hydraulic_force ?? telemetry.controller.brake_force;
    telemetry.controller.drag_force = telemetry.controller.drag_force ?? 0;
    telemetry.controller.rolling_force = telemetry.controller.rolling_force ?? 0;

    telemetry.powertrain.total_torque = telemetry.powertrain.total_torque ?? 0;
    telemetry.powertrain.drive_torque = telemetry.powertrain.drive_torque ?? 0;
    telemetry.powertrain.regen_torque = telemetry.powertrain.regen_torque ?? 0;
    telemetry.powertrain.mechanical_power = telemetry.powertrain.mechanical_power ?? telemetry.powertrain.total_torque * telemetry.velocity.speed;
    telemetry.powertrain.battery_power = telemetry.powertrain.battery_power ?? telemetry.powertrain.mechanical_power;
    telemetry.powertrain.soc = telemetry.powertrain.soc ?? 0.5;

    telemetry.front_axle.drive_torque = telemetry.front_axle.drive_torque ?? 0;
    telemetry.rear_axle.drive_torque = telemetry.rear_axle.drive_torque ?? 0;
    telemetry.front_axle.brake_torque = telemetry.front_axle.brake_torque ?? 0;
    telemetry.rear_axle.brake_torque = telemetry.rear_axle.brake_torque ?? 0;
    telemetry.front_axle.regen_torque = telemetry.front_axle.regen_torque ?? 0;
    telemetry.rear_axle.regen_torque = telemetry.rear_axle.regen_torque ?? 0;
    telemetry.front_axle.normal_force = telemetry.front_axle.normal_force ?? 0;
    telemetry.rear_axle.normal_force = telemetry.rear_axle.normal_force ?? 0;

    const slipRatioFront = telemetry.front_axle.left.slip_ratio ?? telemetry.front_axle.right.slip_ratio ?? 0;
    const slipRatioRear = telemetry.rear_axle.left.slip_ratio ?? telemetry.rear_axle.right.slip_ratio ?? 0;
    telemetry.front_axle.left.slip_ratio = telemetry.front_axle.left.slip_ratio ?? slipRatioFront;
    telemetry.front_axle.right.slip_ratio = telemetry.front_axle.right.slip_ratio ?? slipRatioFront;
    telemetry.rear_axle.left.slip_ratio = telemetry.rear_axle.left.slip_ratio ?? slipRatioRear;
    telemetry.rear_axle.right.slip_ratio = telemetry.rear_axle.right.slip_ratio ?? slipRatioRear;

    telemetry.front_axle.left.friction_utilization = telemetry.front_axle.left.friction_utilization ?? Math.abs(telemetry.front_axle.left.slip_ratio);
    telemetry.front_axle.right.friction_utilization = telemetry.front_axle.right.friction_utilization ?? Math.abs(telemetry.front_axle.right.slip_ratio);
    telemetry.rear_axle.left.friction_utilization = telemetry.rear_axle.left.friction_utilization ?? Math.abs(telemetry.rear_axle.left.slip_ratio);
    telemetry.rear_axle.right.friction_utilization = telemetry.rear_axle.right.friction_utilization ?? Math.abs(telemetry.rear_axle.right.slip_ratio);

    telemetry.totals.distance_traveled_m = this.cumulativeDistance;
    telemetry.totals.energy_consumed_joules = this.cumulativeEnergy;
    telemetry.totals.simulation_time_s = snapshot.simulation_time_s ?? this.simulationTime;
    telemetry.detector_severity = telemetry.detector_severity ?? 0;
    telemetry.safety_stage = telemetry.safety_stage ?? SafetyStage.Normal;
    telemetry.detector_forced = telemetry.detector_forced ?? false;
    telemetry.low_speed_engaged = telemetry.low_speed_engaged ?? false;

    return telemetry;
  }

  private async performReset(params: ResetParams = {}): Promise<void> {
    const originalModel = this.model;
    const originalVehicle = this.vehicleId;
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
    if (!this.init.backend && ((params.model && params.model !== originalModel) || (params.vehicle_id && params.vehicle_id !== originalVehicle))) {
      this.backend = new HybridSimulationBackend({
        model: this.model,
        vehicleId: this.vehicleId,
        configManager: this.configManager,
        nativeFactory: this.init.native_factory,
        driftEnabled: this.driftEnabled,
      });
    }
    const timingInfo = this.init.timing ?? await this.configManager.loadModelTiming(this.model).catch(() => kDefaultTimings[this.model]);
    this.timing = new ModelTiming(timingInfo);
    const schedule = this.timing.planSteps(params.dt ?? timingInfo.nominal_dt);
    this.lastDt = schedule.clamped_dt;
    await (this.backend.ready ?? Promise.resolve());
    await this.backend.reset(params.initial_state ?? [], schedule.substeps[0]);
    this.cumulativeDistance = 0;
    this.cumulativeEnergy = 0;
    this.simulationTime = 0;
    this.lastTelemetry = new SimulationTelemetryState();
  }
}
