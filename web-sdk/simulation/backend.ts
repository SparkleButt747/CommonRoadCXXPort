import { ConfigManager } from '../io/ConfigManager.js';
import { SafetyStage, SimulationTelemetryState } from '../telemetry/index.js';
import { ModelType } from './types.js';
import type { SimulationTelemetry } from '../telemetry/index.js';

export interface BackendSnapshot {
  state: number[];
  telemetry?: SimulationTelemetry;
  dt?: number;
  simulation_time_s?: number;
}

export interface SimulationBackend {
  ready?: Promise<void>;
  reset(state: number[], dt: number): void | Promise<void>;
  step(control: number[], dt: number): void | Promise<void>;
  snapshot(): BackendSnapshot;
  speed(): number;
}

export interface NativeDaemonHandle {
  reset(state: number[], dt: number): void;
  step(control: number[], dt: number): void;
  snapshot(): BackendSnapshot;
  speed(): number;
}

export interface NativeDaemonFactory {
  (options: {
    model: ModelType;
    vehicleParameters: Record<string, unknown>;
    lowSpeedSafety: Record<string, any>;
    lossOfControl: Record<string, any>;
  }): Promise<NativeDaemonHandle>;
}

interface LossOfControlChannelThresholds {
  threshold: number;
  rate: number;
}

interface LossOfControlConfig {
  yaw_rate?: LossOfControlChannelThresholds;
  slip_angle?: LossOfControlChannelThresholds;
  lateral_accel?: LossOfControlChannelThresholds;
  slip_ratio?: LossOfControlChannelThresholds;
}

interface LowSpeedRegimeConfig {
  engage_speed: number;
  release_speed: number;
  yaw_rate_limit: number;
  slip_angle_limit: number;
}

interface LowSpeedSafetyConfig {
  drift_enabled: boolean;
  stop_speed_epsilon: number;
  normal: LowSpeedRegimeConfig;
  drift: LowSpeedRegimeConfig;
}

/**
 * Hybrid backend that prefers a native daemon (e.g., WASM bindings) but
 * transparently falls back to a pure-JS physics approximation when native
 * bindings are unavailable.
 */
export class HybridSimulationBackend implements SimulationBackend {
  private delegate: SimulationBackend;
  ready: Promise<void>;

  constructor(
    private readonly options: {
      model: ModelType;
      vehicleId: number;
      configManager: ConfigManager;
      nativeFactory?: NativeDaemonFactory;
      driftEnabled: boolean;
      useStubBackend?: boolean;
    }
  ) {
    this.delegate = new StubSimulationBackend({ driftEnabled: options.driftEnabled });
    this.ready = this.initialize();
  }

  async reset(state: number[], dt: number): Promise<void> {
    await this.ready;
    return this.delegate.reset(state, dt);
  }

  async step(control: number[], dt: number): Promise<void> {
    await this.ready;
    return this.delegate.step(control, dt);
  }

  snapshot(): BackendSnapshot {
    return this.delegate.snapshot();
  }

  speed(): number {
    return this.delegate.speed();
  }

  private async initialize(): Promise<void> {
    const { configManager, vehicleId, model } = this.options;
    const fallbackDefaults = await this.loadConfigs(configManager, vehicleId, model).catch((error) => {
      console.warn(`HybridSimulationBackend failed to load configs; using defaults: ${error}`);
      return {
        vehicle: {},
        lowSpeed: defaultLowSpeedSafety(),
        loss: defaultLossOfControlConfig(model),
      };
    });

    if (this.options.nativeFactory) {
      const native = await this.options
        .nativeFactory({
          model,
          vehicleParameters: fallbackDefaults.vehicle,
          lowSpeedSafety: fallbackDefaults.lowSpeed,
          lossOfControl: fallbackDefaults.loss,
        })
        .catch((error) => {
          console.warn(`Native daemon unavailable; falling back to JS backend: ${error}`);
          return undefined;
        });
      if (native) {
        this.delegate = new NativeSimulationBackend(native);
        return;
      }
    }

    if (this.options.useStubBackend) {
      console.warn('HybridSimulationBackend using stub backend by explicit request');
      const stub = this.delegate as StubSimulationBackend;
      stub.configure({
        vehicleParameters: fallbackDefaults.vehicle,
        lowSpeedSafety: fallbackDefaults.lowSpeed,
        lossOfControl: fallbackDefaults.loss,
      });
      return;
    }

    this.delegate = new ModelJsSimulationBackend({
      model,
      driftEnabled: this.options.driftEnabled,
      vehicleParameters: fallbackDefaults.vehicle,
      lowSpeedSafety: fallbackDefaults.lowSpeed,
      lossOfControl: fallbackDefaults.loss,
    });
  }

  private async loadConfigs(configManager: ConfigManager, vehicleId: number, model: ModelType) {
    const [vehicle, lowSpeedDoc, lossDoc] = await Promise.all([
      configManager.loadVehicleParameters(vehicleId),
      configManager.loadLowSpeedSafetyConfig(model),
      configManager.loadLossOfControlDetectorConfig(model),
    ]);

    const lowSpeed = parseConfigDocument(lowSpeedDoc) as Record<string, any>;
    const lossRoot = parseConfigDocument(lossDoc) as Record<string, any>;
    const loss = (lossRoot && typeof lossRoot === 'object' && lossRoot[modelKey(model)])
      ? (lossRoot[modelKey(model)] as Record<string, any>)
      : lossRoot;

    return {
      vehicle: ensureObject(vehicle),
      lowSpeed: normalizeLowSpeedSafety(lowSpeed),
      loss: normalizeLossOfControl(loss, model),
    };
  }
}

class NativeSimulationBackend implements SimulationBackend {
  constructor(private readonly handle: NativeDaemonHandle) {}

  reset(state: number[], dt: number): void {
    this.handle.reset(state, dt);
  }

  step(control: number[], dt: number): void {
    this.handle.step(control, dt);
  }

  snapshot(): BackendSnapshot {
    return this.handle.snapshot();
  }

  speed(): number {
    return this.handle.speed();
  }
}

class StubSimulationBackend implements SimulationBackend {
  private telemetry = new SimulationTelemetryState();
  private currentState: number[] = [0, 0, 0, 0];
  private dt = 0;
  private simulationTime = 0;
  private vehicleMass = 1200;
  private lossOfControl: LossOfControlConfig = defaultLossOfControlConfig(ModelType.MB);
  private lowSpeedSafety: LowSpeedSafetyConfig = defaultLowSpeedSafety();
  private driftEnabled: boolean;

  constructor(options: { driftEnabled: boolean }) {
    this.driftEnabled = options.driftEnabled;
  }

  configure(config: {
    vehicleParameters: Record<string, unknown>;
    lowSpeedSafety: LowSpeedSafetyConfig;
    lossOfControl: LossOfControlConfig;
  }): void {
    this.vehicleMass = Number(config.vehicleParameters?.m ?? this.vehicleMass);
    this.lowSpeedSafety = config.lowSpeedSafety;
    this.lossOfControl = config.lossOfControl;
    if (typeof config.lowSpeedSafety?.drift_enabled === 'boolean') {
      this.driftEnabled = config.lowSpeedSafety.drift_enabled;
    }
  }

  reset(state: number[], dt: number): void {
    this.currentState = [state[0] ?? 0, state[1] ?? 0, state[2] ?? 0, state[3] ?? 0];
    this.telemetry = new SimulationTelemetryState();
    this.telemetry.totals.simulation_time_s = 0;
    this.dt = dt;
    this.simulationTime = 0;
  }

  step(control: number[], dt: number): void {
    const accel = control[1] ?? 0;
    const newSpeed = (this.currentState[3] ?? 0) + accel * dt;
    this.currentState[3] = newSpeed;
    this.dt = dt;
    this.simulationTime += dt;
    this.telemetry.totals.simulation_time_s = this.simulationTime;
    this.telemetry.acceleration.longitudinal = accel;
    this.telemetry.controller.acceleration = accel;
    this.telemetry.velocity.speed = newSpeed;
    this.telemetry.velocity.longitudinal = newSpeed;
    this.telemetry.low_speed_engaged = Math.abs(newSpeed) < (this.lowSpeedSafety.stop_speed_epsilon ?? 0.05);
  }

  snapshot(): BackendSnapshot {
    return {
      state: [...this.currentState],
      telemetry: this.telemetry,
      dt: this.dt,
      simulation_time_s: this.simulationTime,
    };
  }

  speed(): number {
    return Number(this.currentState[3] ?? 0);
  }
}

class ModelJsSimulationBackend implements SimulationBackend {
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

  snapshot(): BackendSnapshot {
    return {
      state: [this.pose.x, this.pose.y, this.pose.yaw, this.velocity.speed, this.steering.angle, this.velocity.yaw_rate],
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
    telemetry.powertrain.soc = 0.5;

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
    const lowSpeed = this.driftEnabled ? this.lowSpeedSafety.drift : this.lowSpeedSafety.normal;
    const speed = Math.abs(telemetry.velocity.speed);
    const slip = Math.abs(telemetry.traction.slip_angle);
    const yaw = Math.abs(telemetry.velocity.yaw_rate);
    const shouldEngage =
      speed <= lowSpeed.engage_speed ||
      yaw > lowSpeed.yaw_rate_limit ||
      slip > lowSpeed.slip_angle_limit ||
      speed < this.lowSpeedSafety.stop_speed_epsilon;
    const shouldRelease =
      speed >= lowSpeed.release_speed && yaw < lowSpeed.yaw_rate_limit * 0.9 && slip < lowSpeed.slip_angle_limit * 0.9;
    if (shouldEngage) {
      this.lowSpeedEngaged = true;
    } else if (shouldRelease) {
      this.lowSpeedEngaged = false;
    }

    const lossSeverity = computeLossOfControlSeverity(
      {
        yaw_rate: yaw,
        slip_angle: slip,
        lateral_accel: Math.abs(telemetry.acceleration.lateral),
        slip_ratio: Math.max(
          Math.abs(telemetry.front_axle.left.slip_ratio),
          Math.abs(telemetry.rear_axle.left.slip_ratio),
          Math.abs(telemetry.front_axle.right.slip_ratio),
          Math.abs(telemetry.rear_axle.right.slip_ratio)
        ),
      },
      this.lossOfControl
    );

    const safety_stage = lossSeverity >= 1 ? SafetyStage.Emergency : lossSeverity > 0 ? SafetyStage.Transition : SafetyStage.Normal;

    return {
      detector_severity: lossSeverity,
      detector_forced: false,
      safety_stage,
      low_speed_engaged: this.lowSpeedEngaged,
    };
  }

  private populateWheelTelemetry(
    wheel: SimulationTelemetryState['front_axle']['left'],
    sideSign: number,
    track: number,
    torque: number,
    brake: number
  ): void {
    const speedOffset = 0.5 * track * this.velocity.yaw_rate * sideSign;
    const wheelSpeed = this.velocity.longitudinal + speedOffset;
    wheel.speed = wheelSpeed;
    wheel.slip_ratio = this.computeSlipRatio(wheelSpeed, this.velocity.longitudinal);
    wheel.friction_utilization = Math.min(1, Math.abs(wheel.slip_ratio));
    if (torque > 0 || brake > 0) {
      wheel.friction_utilization = Math.min(1, wheel.friction_utilization + Math.abs(torque + brake) / Math.max(this.vehicleMass, 1));
    }
  }

  private computeSlipRatio(wheelSpeed: number, reference: number): number {
    if (Math.abs(reference) < 1e-6) return 0;
    return (wheelSpeed - reference) / Math.abs(reference);
  }

  private powerFrom(accel: number, speed: number): number {
    const force = accel * this.vehicleMass;
    return force * speed;
  }
}

function computeLossOfControlSeverity(samples: Record<string, number>, config: LossOfControlConfig): number {
  let severity = 0;
  for (const [key, value] of Object.entries(samples)) {
    const channel = (config as Record<string, LossOfControlChannelThresholds>)[key];
    if (!channel) continue;
    if (!Number.isFinite(value) || !Number.isFinite(channel.threshold) || !Number.isFinite(channel.rate)) continue;
    if (value <= channel.threshold) continue;
    const delta = value - channel.threshold;
    severity = Math.max(severity, Math.min(1, delta / Math.max(channel.rate, 1e-6)));
  }
  return severity;
}

function clampNumber(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max);
}

function numberOrFallback(value: unknown, fallback: number): number {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : fallback;
}

function parseConfigDocument(document: unknown): Record<string, unknown> {
  if (typeof document === 'string') {
    return parseYamlLike(document);
  }
  if (document && typeof document === 'object') {
    return document as Record<string, unknown>;
  }
  return {};
}

function parseYamlLike(document: string): Record<string, unknown> {
  const result: Record<string, unknown> = {};
  const lines = document.split(/\r?\n/).filter((line) => line.trim().length > 0 && !line.trim().startsWith('#'));
  const stack: Array<{ indent: number; target: Record<string, unknown> }> = [{ indent: -1, target: result }];

  for (const raw of lines) {
    const match = raw.match(/^(\s*)([^:]+):\s*(.*)$/);
    if (!match) continue;
    const [, spaces, key, value] = match;
    const indent = spaces.length;
    while (stack.length > 1 && indent <= stack[stack.length - 1].indent) {
      stack.pop();
    }
    const parent = stack[stack.length - 1].target;
    if (value === '') {
      const child: Record<string, unknown> = {};
      parent[key.trim()] = child;
      stack.push({ indent, target: child });
    } else {
      const numeric = Number(value);
      parent[key.trim()] = Number.isFinite(numeric) ? numeric : value.trim();
    }
  }
  return result;
}

function ensureObject(value: unknown): Record<string, unknown> {
  if (value && typeof value === 'object') {
    return value as Record<string, unknown>;
  }
  return {};
}

function modelKey(model: ModelType): string {
  switch (model) {
    case ModelType.MB:
      return 'mb';
    case ModelType.ST:
      return 'st';
    case ModelType.STD:
      return 'std';
    default:
      return `${model}`;
  }
}

function normalizeLowSpeedSafety(raw: Record<string, any>): LowSpeedSafetyConfig {
  const base = defaultLowSpeedSafety();
  const merged: LowSpeedSafetyConfig = {
    ...base,
    drift_enabled: coerceBoolean(raw.drift_enabled, base.drift_enabled),
    stop_speed_epsilon: Number(raw.stop_speed_epsilon ?? base.stop_speed_epsilon),
    normal: { ...base.normal },
    drift: { ...base.drift },
  };
  if (raw.normal) {
    merged.normal = {
      ...merged.normal,
      engage_speed: Number(raw.normal.engage_speed ?? merged.normal.engage_speed),
      release_speed: Number(raw.normal.release_speed ?? merged.normal.release_speed),
      yaw_rate_limit: Number(raw.normal.yaw_rate_limit ?? merged.normal.yaw_rate_limit),
      slip_angle_limit: Number(raw.normal.slip_angle_limit ?? merged.normal.slip_angle_limit),
    };
  }
  if (raw.drift) {
    merged.drift = {
      ...merged.drift,
      engage_speed: Number(raw.drift.engage_speed ?? merged.drift.engage_speed),
      release_speed: Number(raw.drift.release_speed ?? merged.drift.release_speed),
      yaw_rate_limit: Number(raw.drift.yaw_rate_limit ?? merged.drift.yaw_rate_limit),
      slip_angle_limit: Number(raw.drift.slip_angle_limit ?? merged.drift.slip_angle_limit),
    };
  }
  return merged;
}

function normalizeLossOfControl(raw: Record<string, any>, model: ModelType): LossOfControlConfig {
  const defaults = defaultLossOfControlConfig(model);
  const result: LossOfControlConfig = { ...defaults };
  for (const key of ['yaw_rate', 'slip_angle', 'lateral_accel', 'slip_ratio']) {
    const channel = raw?.[key];
    if (channel && typeof channel === 'object') {
      const threshold = Number(channel.threshold ?? result[key as keyof LossOfControlConfig]?.threshold ?? 0);
      const rate = Number(channel.rate ?? result[key as keyof LossOfControlConfig]?.rate ?? 1);
      (result as any)[key] = { threshold, rate };
    }
  }
  return result;
}

function coerceBoolean(value: any, fallback: boolean): boolean {
  if (typeof value === 'boolean') return value;
  if (value === 'true') return true;
  if (value === 'false') return false;
  return fallback;
}

function defaultLowSpeedSafety(): LowSpeedSafetyConfig {
  return {
    drift_enabled: false,
    stop_speed_epsilon: 0.05,
    normal: { engage_speed: 0.4, release_speed: 0.8, yaw_rate_limit: 0.5, slip_angle_limit: 0.35 },
    drift: { engage_speed: 0.3, release_speed: 1.0, yaw_rate_limit: 0.8, slip_angle_limit: 0.6 },
  };
}

function defaultLossOfControlConfig(model: ModelType): LossOfControlConfig {
  switch (model) {
    case ModelType.ST:
      return {
        yaw_rate: { threshold: 1.2, rate: 9 },
        slip_angle: { threshold: 0.45, rate: 4.5 },
        lateral_accel: { threshold: 6.5, rate: 12 },
        slip_ratio: { threshold: 0.18, rate: 6 },
      };
    case ModelType.STD:
      return {
        yaw_rate: { threshold: 1.4, rate: 10 },
        slip_angle: { threshold: 0.8, rate: 5 },
        lateral_accel: { threshold: 7.0, rate: 14 },
        slip_ratio: { threshold: 0.5, rate: 7 },
      };
    case ModelType.MB:
    default:
      return {
        yaw_rate: { threshold: 1.6, rate: 10 },
        slip_angle: { threshold: 0.5, rate: 5 },
        lateral_accel: { threshold: 7.5, rate: 15 },
        slip_ratio: { threshold: 0.2, rate: 7 },
      };
  }
}
