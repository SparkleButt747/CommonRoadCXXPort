import { MBSimulationBackend, STSimulationBackend, STDSimulationBackend } from './vehicleBackends.js';
import type { NativeDaemonFactory, NativeDaemonHandle } from './backend.js';
import { ModelType } from './types.js';

class TypeScriptNativeDaemon implements NativeDaemonHandle {
  constructor(
    private readonly backend: MBSimulationBackend | STSimulationBackend | STDSimulationBackend
  ) {}

  reset(state: number[], dt: number): void {
    this.backend.reset(state, dt);
  }

  step(control: number[], dt: number): void {
    this.backend.step(control, dt);
  }

  snapshot() {
    return this.backend.snapshot();
  }

  speed(): number {
    return this.backend.speed();
  }
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

function selectModelConfig<T extends Record<string, any>>(config: T, model: ModelType): T {
  if (!config || typeof config !== 'object') {
    return {} as T;
  }
  const key = modelKey(model);
  const candidate = (config as Record<string, any>)[key];
  if (candidate && typeof candidate === 'object') {
    return candidate as T;
  }
  return config;
}

/**
 * Packaged native daemon factory implemented purely in TypeScript. This mirrors
 * the C++ solvers and safety logic so that HybridSimulationBackend can run the
 * MB, ST, and STD models without external bindings.
 */
export const packagedNativeFactory: NativeDaemonFactory = async ({
  model,
  vehicleParameters,
  lowSpeedSafety,
  lossOfControl,
}) => {
  const safetyConfig = selectModelConfig(lowSpeedSafety ?? {}, model);
  const lossConfig = selectModelConfig(lossOfControl ?? {}, model);
  const driftEnabled = Boolean((safetyConfig as any).drift_enabled);

  const backend = createModelBackend({
    model,
    params: vehicleParameters ?? {},
    lowSpeed: safetyConfig as any,
    lossConfig: lossConfig as any,
    driftEnabled,
  });

  return new TypeScriptNativeDaemon(backend);
};

function createModelBackend(options: {
  model: ModelType;
  params: Record<string, unknown>;
  lowSpeed: Record<string, unknown>;
  lossConfig: Record<string, unknown>;
  driftEnabled: boolean;
}): MBSimulationBackend | STSimulationBackend | STDSimulationBackend {
  switch (options.model) {
    case ModelType.ST:
      return new STSimulationBackend(options);
    case ModelType.STD:
      return new STDSimulationBackend(options);
    case ModelType.MB:
    default:
      return new MBSimulationBackend(options);
  }
}

export default packagedNativeFactory;
