import { ModelTimingInfo, ModelType } from '../simulation/types.js';

export type Fetcher = (input: RequestInfo | URL, init?: RequestInit) => Promise<Response>;

function withTrailingSlash(path: string): string {
  return path.endsWith('/') ? path : `${path}/`;
}

function siblingConfigRoot(parameterRoot: string): string {
  const trimmed = parameterRoot.replace(/\/+$/, '');
  const segments = trimmed.split('/');
  segments.pop();
  segments.push('config');
  return segments.join('/') || 'config';
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

async function ensureAvailable(fetcher: Fetcher, url: string, description: string): Promise<void> {
  const response = await fetcher(url, { method: 'HEAD' }).catch((error) => {
    throw new Error(`${description} root ${url} is unreachable: ${error}`);
  });
  if (!response.ok) {
    throw new Error(`${description} root ${url} is not accessible (status ${response.status})`);
  }
}

export class ConfigManager {
  readonly configRoot: string;
  readonly parameterRoot: string;
  private readonly fetcher: Fetcher;
  private rootsChecked = false;

  constructor(configRoot?: string, parameterRoot: string = 'parameters', fetcher?: Fetcher) {
    this.fetcher = fetcher ?? (typeof fetch !== 'undefined' ? fetch.bind(globalThis) : undefined as unknown as Fetcher);
    if (!this.fetcher) {
      throw new Error('A fetch-compatible API is required to load configuration assets.');
    }

    const paramRoot = withTrailingSlash(parameterRoot);
    const derivedConfig = withTrailingSlash(configRoot ?? siblingConfigRoot(paramRoot));
    this.parameterRoot = paramRoot;
    this.configRoot = derivedConfig;
  }

  async verifyRoots(): Promise<void> {
    if (this.rootsChecked) {
      return;
    }
    await ensureAvailable(this.fetcher, this.parameterRoot, 'Parameter');
    await ensureAvailable(this.fetcher, this.configRoot, 'Config');
    this.rootsChecked = true;
  }

  async loadVehicleParameters(vehicleId: number): Promise<unknown> {
    await this.verifyRoots();
    const path = this.resolveParameterPath(`vehicle/parameters_vehicle${vehicleId}.yaml`);
    return this.fetchDocument(path, `vehicle parameters for id ${vehicleId}`);
  }

  async loadAeroConfig(path = 'aero.yaml'): Promise<unknown> {
    await this.verifyRoots();
    return this.fetchDocument(this.resolveConfigPath(path), 'aero config');
  }

  async loadRollingResistanceConfig(path = 'rolling.yaml'): Promise<unknown> {
    await this.verifyRoots();
    return this.fetchDocument(this.resolveConfigPath(path), 'rolling resistance config');
  }

  async loadBrakeConfig(path = 'brakes.yaml'): Promise<unknown> {
    await this.verifyRoots();
    return this.fetchDocument(this.resolveConfigPath(path), 'brake config');
  }

  async loadPowertrainConfig(path = 'powertrain.yaml'): Promise<unknown> {
    await this.verifyRoots();
    return this.fetchDocument(this.resolveConfigPath(path), 'powertrain config');
  }

  async loadFinalAccelControllerConfig(path = 'final_accel_controller.yaml'): Promise<unknown> {
    await this.verifyRoots();
    return this.fetchDocument(this.resolveConfigPath(path), 'final accel controller config');
  }

  async loadSteeringConfig(path = 'steering.yaml'): Promise<unknown> {
    await this.verifyRoots();
    return this.fetchDocument(this.resolveConfigPath(path), 'steering config');
  }

  async loadLowSpeedSafetyConfig(model: ModelType): Promise<unknown> {
    await this.verifyRoots();
    const suffix = modelKey(model);
    const overrideName = `low_speed_safety_${suffix}.yaml`;
    const overridePath = this.resolveConfigPath(overrideName);
    const fallback = this.resolveConfigPath('low_speed_safety.yaml');
    try {
      return await this.fetchDocument(overridePath, `low speed safety config (${suffix})`);
    } catch (error) {
      return this.fetchDocument(fallback, 'low speed safety config (default)');
    }
  }

  async loadLossOfControlDetectorConfig(model: ModelType): Promise<unknown> {
    await this.verifyRoots();
    const key = modelKey(model);
    return this.fetchDocument(this.resolveConfigPath('loss_of_control_detector.yaml'), `loss of control detector (${key})`);
  }

  async loadModelTiming(model: ModelType): Promise<ModelTimingInfo> {
    await this.verifyRoots();
    const defaultTimings: Record<ModelType, ModelTimingInfo> = {
      [ModelType.MB]: { nominal_dt: 0.005, max_dt: 0.005 },
      [ModelType.ST]: { nominal_dt: 0.01, max_dt: 0.02 },
      [ModelType.STD]: { nominal_dt: 0.01, max_dt: 0.01 },
    };
    const path = this.resolveConfigPath('model_timing.yaml');
    try {
      const document = await this.fetchDocument(path, 'model timing');
      if (typeof document === 'string') {
        const parsed = this.parseYamlLike(document);
        const modelSection = (parsed as Record<string, any>)[modelKey(model)];
        if (modelSection && typeof modelSection === 'object') {
          const nominal = Number(modelSection.nominal_dt);
          const max = Number(modelSection.max_dt);
          if (Number.isFinite(nominal) && Number.isFinite(max)) {
            return { nominal_dt: nominal, max_dt: max };
          }
        }
      }
    } catch (error) {
      // Fall back to defaults when timing overrides are unavailable.
      console.warn(`Timing config missing or invalid at ${path}: ${error}`);
    }
    return defaultTimings[model];
  }

  private resolveConfigPath(path: string): string {
    return new URL(path, this.configRoot).toString();
  }

  private resolveParameterPath(path: string): string {
    return new URL(path, this.parameterRoot).toString();
  }

  private async fetchDocument(path: string, description: string): Promise<unknown> {
    const response = await this.fetcher(path).catch((error) => {
      throw new Error(`Failed to fetch ${description} at ${path}: ${error}`);
    });
    if (!response.ok) {
      throw new Error(`Missing ${description} at ${path} (status ${response.status})`);
    }
    const contentType = response.headers.get('content-type') ?? '';
    const body = await response.text();
    const looksJson = contentType.includes('application/json') || path.endsWith('.json');
    if (looksJson) {
      try {
        return JSON.parse(body);
      } catch (error) {
        throw new Error(`Invalid JSON in ${description} at ${path}: ${error}`);
      }
    }
    return body;
  }

  private parseYamlLike(document: string): Record<string, unknown> {
    try {
      const jsonCandidate = JSON.parse(document);
      if (jsonCandidate && typeof jsonCandidate === 'object') {
        return jsonCandidate as Record<string, unknown>;
      }
    } catch (error) {
      // Fall through to very small YAML subset.
    }
    const lines = document.split(/\r?\n/).filter((line) => line.trim().length > 0 && !line.trim().startsWith('#'));
    const result: Record<string, unknown> = {};
    for (const line of lines) {
      const [key, value] = line.split(':').map((part) => part.trim());
      if (!key) continue;
      const numeric = Number(value);
      result[key] = Number.isFinite(numeric) ? numeric : value;
    }
    return result;
  }
}
