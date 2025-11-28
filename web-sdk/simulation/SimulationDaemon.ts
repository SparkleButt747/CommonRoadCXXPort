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
import { SteeringWheel, FinalSteerController } from '../controllers/steering.js';
import { FinalAccelController, ControllerOutput as AccelControllerOutput } from '../controllers/longitudinal/finalAccelController.js';
import { VehicleParameters } from '../models/types.js';
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
  private params?: VehicleParameters;
  private steeringWheel?: SteeringWheel;
  private finalSteer?: FinalSteerController;
  private accelController?: FinalAccelController;
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
    let accelOutput: AccelControllerOutput | undefined;
    let steeringOutput: SteeringWheel.Output | undefined;
    let finalOutput: FinalSteerController.Output | undefined;
    for (const dt of schedule.substeps) {
      if (sanitized.control_mode === ControlMode.Keyboard &&
          this.accelController && this.steeringWheel && this.finalSteer) {
        const startSpeed = this.backend.speed();
        const state = this.backend.snapshot().state ?? [];
        const currentAngle = state[2] ?? 0;
        steeringOutput = this.steeringWheel.update(sanitized.steering_nudge ?? 0, dt);
        finalOutput = this.finalSteer.update(steeringOutput.target_angle, currentAngle, dt);
        accelOutput = this.accelController.step(sanitized.longitudinal as DriverIntent, startSpeed, dt);
        await this.backend.step([finalOutput.rate, accelOutput.acceleration], dt);
        const endSpeed = this.backend.speed();
        const meanSpeed = 0.5 * (Math.abs(startSpeed) + Math.abs(endSpeed));
        const batteryPower = accelOutput.battery_power ?? accelOutput.acceleration * meanSpeed * (this.params?.m ?? 0);
        this.cumulativeDistance += meanSpeed * dt;
        this.cumulativeEnergy += batteryPower * dt;
      } else if (sanitized.control_mode === ControlMode.Direct) {
        accelCommand = this.directAccelerationFromTorque(sanitized.axle_torques ?? []);
        steerRate = 0;
        const control = [steerRate, accelCommand];
        await this.backend.step(control, dt);
        const speed = this.backend.speed();
        this.cumulativeDistance += Math.abs(speed) * dt;
        this.cumulativeEnergy += accelCommand * speed * dt;
      } else {
        accelCommand = sanitized.longitudinal.throttle - sanitized.longitudinal.brake;
        const nudge = sanitized.steering_nudge ?? 0;
        steerRate = nudge;
        const control = [steerRate, accelCommand];
        await this.backend.step(control, dt);
        const speed = this.backend.speed();
        this.cumulativeDistance += Math.abs(speed) * dt;
        this.cumulativeEnergy += accelCommand * speed * dt;
      }
      this.simulationTime += dt;
    }

    this.lastTelemetry = this.buildTelemetry(this.backend.snapshot(), accelOutput, steeringOutput, finalOutput);
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

  private buildTelemetry(
    snapshot: BackendSnapshot,
    accelOutput?: AccelControllerOutput,
    steeringInput?: SteeringWheel.Output,
    steeringOutput?: FinalSteerController.Output
  ): SimulationTelemetryState {
    const telemetry = snapshot.telemetry ? mergeTelemetry(snapshot.telemetry) : new SimulationTelemetryState();

    if (snapshot.telemetry) {
      this.cumulativeDistance = telemetry.totals.distance_traveled_m ?? this.cumulativeDistance;
      this.cumulativeEnergy = telemetry.totals.energy_consumed_joules ?? this.cumulativeEnergy;
      this.simulationTime = snapshot.simulation_time_s ?? telemetry.totals.simulation_time_s ?? this.simulationTime;
    } else {
      const [x, y, yaw, speed = this.backend.speed()] = snapshot.state;
      telemetry.pose.x = x ?? telemetry.pose.x;
      telemetry.pose.y = y ?? telemetry.pose.y;
      telemetry.pose.yaw = yaw ?? telemetry.pose.yaw;
      telemetry.velocity.speed = speed ?? telemetry.velocity.speed;
      telemetry.velocity.longitudinal = telemetry.velocity.longitudinal ?? telemetry.velocity.speed;
      telemetry.velocity.yaw_rate = snapshot.state[5] ?? telemetry.velocity.yaw_rate;
      telemetry.traction.slip_angle = snapshot.state[6] ?? telemetry.traction.slip_angle;
      telemetry.steering.actual_angle = snapshot.state[7] ?? telemetry.steering.actual_angle;
    }

    if (accelOutput) {
      telemetry.controller.acceleration = accelOutput.acceleration;
      telemetry.controller.throttle = accelOutput.throttle;
      telemetry.controller.brake = accelOutput.brake;
      telemetry.controller.drive_force = accelOutput.drive_force;
      telemetry.controller.brake_force = accelOutput.brake_force;
      telemetry.controller.regen_force = accelOutput.regen_force;
      telemetry.controller.hydraulic_force = accelOutput.hydraulic_force;
      telemetry.controller.drag_force = accelOutput.drag_force;
      telemetry.controller.rolling_force = accelOutput.rolling_force;

      telemetry.powertrain.drive_torque = accelOutput.drive_force * (this.params?.R_w ?? 0);
      telemetry.powertrain.regen_torque = accelOutput.regen_force * (this.params?.R_w ?? 0);
      telemetry.powertrain.total_torque = telemetry.powertrain.drive_torque - telemetry.powertrain.regen_torque;
      telemetry.powertrain.mechanical_power = accelOutput.mechanical_power;
      telemetry.powertrain.battery_power = accelOutput.battery_power;
      telemetry.powertrain.soc = accelOutput.soc;
    }
    if (steeringInput) {
      telemetry.steering.desired_angle = steeringInput.target_angle;
      telemetry.steering.actual_angle = steeringInput.angle;
      telemetry.steering.desired_rate = steeringInput.rate;
      telemetry.steering.actual_rate = steeringInput.rate;
    }
    if (steeringOutput) {
      telemetry.steering.actual_angle = steeringOutput.angle;
      telemetry.steering.desired_angle = steeringOutput.filtered_target;
      telemetry.steering.actual_rate = steeringOutput.rate;
      telemetry.steering.desired_rate = steeringOutput.rate;
    }

    telemetry.totals.distance_traveled_m = telemetry.totals.distance_traveled_m ?? this.cumulativeDistance;
    telemetry.totals.energy_consumed_joules = telemetry.totals.energy_consumed_joules ?? this.cumulativeEnergy;
    telemetry.totals.simulation_time_s = snapshot.simulation_time_s ?? telemetry.totals.simulation_time_s ?? this.simulationTime;

    return telemetry;
  }

  private async rebuildControllers(): Promise<void> {
    if (!this.params) {
      this.params = await this.configManager.loadVehicleParameters(this.vehicleId).catch(() => this.params);
    }
    if (!this.params) {
      return;
    }
    const steeringCfg = await this.configManager.loadSteeringConfig().catch(() => ({} as any));
    if (steeringCfg?.wheel && steeringCfg?.final) {
      this.steeringWheel = new SteeringWheel(steeringCfg.wheel, this.params.steering);
      this.finalSteer = new FinalSteerController(steeringCfg.final, this.params.steering);
    }

    const aeroCfg = await this.configManager.loadAeroConfig().catch(() => ({ drag_coefficient: 0, downforce_coefficient: 0 }));
    const rollingCfg = await this.configManager.loadRollingResistanceConfig().catch(() => ({ c_rr: 0 }));
    const brakeCfg = await this.configManager.loadBrakeConfig().catch(() => ({ max_force: 0, max_regen_force: 0, min_regen_speed: 0 }));
    const powertrainCfg = await this.configManager.loadPowertrainConfig().catch(() => ({
      max_drive_torque: 0,
      max_regen_torque: 0,
      max_power: 0,
      drive_efficiency: 1,
      regen_efficiency: 0,
      min_soc: 0,
      max_soc: 1,
      initial_soc: 0.5,
      battery_capacity_kwh: 1,
    }));
    const accelCfg = await this.configManager.loadFinalAccelControllerConfig().catch(() => ({
      tau_throttle: 0.1,
      tau_brake: 0.1,
      accel_min: -5,
      accel_max: 5,
      stop_speed_epsilon: 0.05,
    }));
    this.accelController = new FinalAccelController(
      this.params.m,
      this.params.R_w,
      powertrainCfg as any,
      aeroCfg as any,
      rollingCfg as any,
      brakeCfg as any,
      accelCfg as any
    );
  }

  private directAccelerationFromTorque(axleTorques: number[]): number {
    if (!this.params) {
      return (axleTorques ?? []).reduce((sum, torque) => sum + torque, 0);
    }
    const frontSplit = Math.min(Math.max(this.params.T_se, 0), 1);
    const drivenAxles = [];
    if (frontSplit > 0) drivenAxles.push('front');
    if (1 - frontSplit > 0) drivenAxles.push('rear');
    if (drivenAxles.length === 0) {
      throw new Error('No driven axles configured for direct torque control');
    }
    if (axleTorques.length !== drivenAxles.length) {
      throw new Error(`Expected ${drivenAxles.length} driven axle torques, got ${axleTorques.length}`);
    }
    if (!(this.params.m > 0)) {
      throw new Error('Vehicle mass must be positive for torque conversion');
    }
    if (!(this.params.R_w > 0)) {
      throw new Error('Wheel radius must be positive for torque conversion');
    }
    const totalTorque = axleTorques.reduce((sum, torque) => sum + torque, 0);
    return totalTorque / (this.params.m * this.params.R_w);
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
    this.params = await this.configManager.loadVehicleParameters(this.vehicleId).catch(() => this.params);
    if (!this.init.backend && ((params.model && params.model !== originalModel) || (params.vehicle_id && params.vehicle_id !== originalVehicle))) {
      this.backend = new HybridSimulationBackend({
        model: this.model,
        vehicleId: this.vehicleId,
        configManager: this.configManager,
        nativeFactory: this.init.native_factory,
        driftEnabled: this.driftEnabled,
      });
    }
    await this.rebuildControllers();
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
