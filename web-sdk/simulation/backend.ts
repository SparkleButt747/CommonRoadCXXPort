import { ConfigManager } from '../io/ConfigManager.js';
import { SimulationTelemetryState } from '../telemetry/index.js';
import { ModelType } from './types.js';
import packagedNativeFactory from './nativeFactory.js';
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
    }
  ) {
    this.delegate = new NativeSimulationBackend(
      new NullDaemonHandle({ driftEnabled: options.driftEnabled })
    );
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

    const factory = this.options.nativeFactory ?? packagedNativeFactory;
    const native = await factory({
      model,
      vehicleParameters: fallbackDefaults.vehicle,
      lowSpeedSafety: fallbackDefaults.lowSpeed,
      lossOfControl: fallbackDefaults.loss,
    });
    this.delegate = new NativeSimulationBackend(native);
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

class NullDaemonHandle implements NativeDaemonHandle {
  private current: BackendSnapshot = {
    state: [0, 0, 0, 0],
    telemetry: new SimulationTelemetryState(),
    dt: 0,
    simulation_time_s: 0,
  };

  constructor(private readonly options: { driftEnabled: boolean }) {
    this.current.telemetry!.traction.drift_mode = options.driftEnabled;
  }

  reset(state: number[], dt: number): void {
    this.current = {
      state: [...state],
      dt,
      simulation_time_s: 0,
      telemetry: new SimulationTelemetryState(),
    };
    this.current.telemetry!.traction.drift_mode = this.options.driftEnabled;
  }

  step(control: number[], dt: number): void {
    const accel = control[1] ?? 0;
    const speed = (this.current.state[3] ?? 0) + accel * dt;
    this.current.state[3] = speed;
    this.current.dt = dt;
    this.current.simulation_time_s = (this.current.simulation_time_s ?? 0) + dt;
    this.current.telemetry!.velocity.speed = speed;
    this.current.telemetry!.controller.acceleration = accel;
    this.current.telemetry!.totals.simulation_time_s = this.current.simulation_time_s;
  }

  snapshot(): BackendSnapshot {
    return this.current;
  }

  speed(): number {
    return Number(this.current.state[3] ?? 0);
  }
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
