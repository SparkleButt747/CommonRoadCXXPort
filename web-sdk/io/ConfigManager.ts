import { VehicleParameters, LongitudinalParameters, SteeringParameters, TireParameters, TrailerParameters } from '../models/types.js';
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

function parseScalar(value: string): any {
  const numeric = Number(value);
  if (Number.isFinite(numeric)) return numeric;
  if (value === 'true') return true;
  if (value === 'false') return false;
  return value;
}

function parseYamlLike(document: string): Record<string, any> {
  const result: Record<string, any> = {};
  const lines = document
    .split(/\r?\n/)
    .filter((line) => line.trim().length > 0 && !line.trim().startsWith('#'));
  const stack: Array<{ indent: number; target: Record<string, any> }> = [{ indent: -1, target: result }];

  for (const raw of lines) {
    const match = raw.match(/^(\s*)([^:]+):\s*(.*)$/);
    if (!match) continue;
    const [, spaces, keyRaw, valueRaw] = match;
    const key = keyRaw.trim();
    const valuePart = valueRaw.replace(/\s+#.*$/, '').trim();
    const indent = spaces.length;
    while (stack.length > 1 && indent <= stack[stack.length - 1].indent) {
      stack.pop();
    }
    const parent = stack[stack.length - 1].target;
    if (valuePart === '') {
      const child: Record<string, any> = {};
      parent[key] = child;
      stack.push({ indent, target: child });
    } else {
      parent[key] = parseScalar(valuePart);
    }
  }
  return result;
}

function ensureNumber(value: any, fallback = 0): number {
  const num = Number(value);
  return Number.isFinite(num) ? num : fallback;
}

function normalizeSteering(raw: any = {}): SteeringParameters {
  return {
    min: ensureNumber(raw.min),
    max: ensureNumber(raw.max),
    v_min: ensureNumber(raw.v_min),
    v_max: ensureNumber(raw.v_max),
    kappa_dot_max: ensureNumber(raw.kappa_dot_max),
    kappa_dot_dot_max: ensureNumber(raw.kappa_dot_dot_max),
  };
}

function normalizeLongitudinal(raw: any = {}): LongitudinalParameters {
  return {
    v_min: ensureNumber(raw.v_min),
    v_max: ensureNumber(raw.v_max),
    v_switch: ensureNumber(raw.v_switch),
    a_max: ensureNumber(raw.a_max),
    j_max: ensureNumber(raw.j_max),
    j_dot_max: ensureNumber(raw.j_dot_max),
  };
}

function normalizeTrailer(raw: any = {}): TrailerParameters {
  return {
    l: ensureNumber(raw.l),
    w: ensureNumber(raw.w),
    l_hitch: ensureNumber(raw.l_hitch),
    l_total: ensureNumber(raw.l_total),
    l_wb: ensureNumber(raw.l_wb),
  };
}

function normalizeTire(raw: any = {}): TireParameters {
  const src = raw.tire && typeof raw.tire === 'object' ? raw.tire : raw;
  return {
    p_cx1: ensureNumber(src.p_cx1),
    p_dx1: ensureNumber(src.p_dx1),
    p_dx3: ensureNumber(src.p_dx3),
    p_ex1: ensureNumber(src.p_ex1),
    p_kx1: ensureNumber(src.p_kx1),
    p_hx1: ensureNumber(src.p_hx1),
    p_vx1: ensureNumber(src.p_vx1),
    r_bx1: ensureNumber(src.r_bx1),
    r_bx2: ensureNumber(src.r_bx2),
    r_cx1: ensureNumber(src.r_cx1),
    r_ex1: ensureNumber(src.r_ex1),
    r_hx1: ensureNumber(src.r_hx1),
    p_cy1: ensureNumber(src.p_cy1),
    p_dy1: ensureNumber(src.p_dy1),
    p_dy3: ensureNumber(src.p_dy3),
    p_ey1: ensureNumber(src.p_ey1),
    p_ky1: ensureNumber(src.p_ky1),
    p_hy1: ensureNumber(src.p_hy1),
    p_hy3: ensureNumber(src.p_hy3),
    p_vy1: ensureNumber(src.p_vy1),
    p_vy3: ensureNumber(src.p_vy3),
    r_by1: ensureNumber(src.r_by1),
    r_by2: ensureNumber(src.r_by2),
    r_by3: ensureNumber(src.r_by3),
    r_cy1: ensureNumber(src.r_cy1),
    r_ey1: ensureNumber(src.r_ey1),
    r_hy1: ensureNumber(src.r_hy1),
    r_vy1: ensureNumber(src.r_vy1),
    r_vy3: ensureNumber(src.r_vy3),
    r_vy4: ensureNumber(src.r_vy4),
    r_vy5: ensureNumber(src.r_vy5),
    r_vy6: ensureNumber(src.r_vy6),
  };
}

function normalizeVehicle(raw: any, tire: TireParameters): VehicleParameters {
  const steering = normalizeSteering(raw?.steering ?? {});
  const longitudinal = normalizeLongitudinal(raw?.longitudinal ?? {});
  const trailer = normalizeTrailer(raw?.trailer ?? {});
  return {
    l: ensureNumber(raw?.l),
    w: ensureNumber(raw?.w),
    steering,
    longitudinal,
    m: ensureNumber(raw?.m),
    m_s: ensureNumber(raw?.m_s),
    m_uf: ensureNumber(raw?.m_uf),
    m_ur: ensureNumber(raw?.m_ur),
    a: ensureNumber(raw?.a),
    b: ensureNumber(raw?.b),
    I_Phi_s: ensureNumber(raw?.I_Phi_s),
    I_y_s: ensureNumber(raw?.I_y_s),
    I_z: ensureNumber(raw?.I_z),
    I_xz_s: ensureNumber(raw?.I_xz_s),
    K_sf: ensureNumber(raw?.K_sf),
    K_sdf: ensureNumber(raw?.K_sdf),
    K_sr: ensureNumber(raw?.K_sr),
    K_sdr: ensureNumber(raw?.K_sdr),
    T_f: ensureNumber(raw?.T_f),
    T_r: ensureNumber(raw?.T_r),
    K_ras: ensureNumber(raw?.K_ras),
    K_tsf: ensureNumber(raw?.K_tsf),
    K_tsr: ensureNumber(raw?.K_tsr),
    K_rad: ensureNumber(raw?.K_rad),
    K_zt: ensureNumber(raw?.K_zt),
    h_cg: ensureNumber(raw?.h_cg),
    h_raf: ensureNumber(raw?.h_raf),
    h_rar: ensureNumber(raw?.h_rar),
    h_s: ensureNumber(raw?.h_s),
    I_uf: ensureNumber(raw?.I_uf),
    I_ur: ensureNumber(raw?.I_ur),
    I_y_w: ensureNumber(raw?.I_y_w),
    K_lt: ensureNumber(raw?.K_lt),
    R_w: ensureNumber(raw?.R_w),
    T_sb: ensureNumber(raw?.T_sb),
    T_se: ensureNumber(raw?.T_se),
    D_f: ensureNumber(raw?.D_f),
    D_r: ensureNumber(raw?.D_r),
    E_f: ensureNumber(raw?.E_f),
    E_r: ensureNumber(raw?.E_r),
    tire,
    trailer,
  };
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
    if (this.rootsChecked) return;
    await ensureAvailable(this.fetcher, this.parameterRoot, 'Parameter');
    await ensureAvailable(this.fetcher, this.configRoot, 'Config');
    this.rootsChecked = true;
  }

  async loadVehicleParameters(vehicleId: number): Promise<VehicleParameters> {
    await this.verifyRoots();
    const vehiclePath = this.resolveParameterPath(`vehicle/parameters_vehicle${vehicleId}.yaml`);
    const tirePath = this.resolveParameterPath('tire/parameters_tire.yaml');
    const [vehicleDoc, tireDoc] = await Promise.all([
      this.fetchDocument(vehiclePath, `vehicle parameters for id ${vehicleId}`),
      this.fetchDocument(tirePath, 'tire parameters'),
    ]);
    const vehicleObj = this.parseDocument(vehicleDoc);
    const tireObj = this.parseDocument(tireDoc);
    const tire = normalizeTire(tireObj);
    return normalizeVehicle(vehicleObj, tire);
  }

  async loadAeroConfig(path = 'aero.yaml'): Promise<Record<string, any>> {
    await this.verifyRoots();
    return this.parseDocument(await this.fetchDocument(this.resolveConfigPath(path), 'aero config'));
  }

  async loadRollingResistanceConfig(path = 'rolling.yaml'): Promise<Record<string, any>> {
    await this.verifyRoots();
    return this.parseDocument(await this.fetchDocument(this.resolveConfigPath(path), 'rolling resistance config'));
  }

  async loadBrakeConfig(path = 'brakes.yaml'): Promise<Record<string, any>> {
    await this.verifyRoots();
    return this.parseDocument(await this.fetchDocument(this.resolveConfigPath(path), 'brake config'));
  }

  async loadPowertrainConfig(path = 'powertrain.yaml'): Promise<Record<string, any>> {
    await this.verifyRoots();
    return this.parseDocument(await this.fetchDocument(this.resolveConfigPath(path), 'powertrain config'));
  }

  async loadFinalAccelControllerConfig(path = 'final_accel_controller.yaml'): Promise<Record<string, any>> {
    await this.verifyRoots();
    return this.parseDocument(await this.fetchDocument(this.resolveConfigPath(path), 'final accel controller config'));
  }

  async loadSteeringConfig(path = 'steering.yaml'): Promise<Record<string, any>> {
    await this.verifyRoots();
    return this.parseDocument(await this.fetchDocument(this.resolveConfigPath(path), 'steering config'));
  }

  async loadLowSpeedSafetyConfig(model: ModelType): Promise<Record<string, any>> {
    await this.verifyRoots();
    const suffix = modelKey(model);
    const overrideName = `low_speed_safety_${suffix}.yaml`;
    const overridePath = this.resolveConfigPath(overrideName);
    const fallback = this.resolveConfigPath('low_speed_safety.yaml');
    try {
      return this.parseDocument(await this.fetchDocument(overridePath, `low speed safety config (${suffix})`));
    } catch (error) {
      return this.parseDocument(await this.fetchDocument(fallback, 'low speed safety config (default)'));
    }
  }

  async loadLossOfControlDetectorConfig(model: ModelType): Promise<Record<string, any>> {
    await this.verifyRoots();
    const key = modelKey(model);
    return this.parseDocument(
      await this.fetchDocument(this.resolveConfigPath('loss_of_control_detector.yaml'), `loss of control detector (${key})`)
    );
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
      const parsed = this.parseDocument(document);
      const modelSection = parsed[modelKey(model)];
      if (modelSection && typeof modelSection === 'object') {
        const nominal = Number((modelSection as any).nominal_dt);
        const max = Number((modelSection as any).max_dt);
        if (Number.isFinite(nominal) && Number.isFinite(max)) {
          return { nominal_dt: nominal, max_dt: max };
        }
      }
    } catch (error) {
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

  private parseDocument(document: unknown): any {
    if (typeof document === 'string') {
      try {
        return parseYamlLike(document);
      } catch (error) {
        throw new Error(`Failed to parse YAML: ${error}`);
      }
    }
    if (document && typeof document === 'object') {
      return document as Record<string, any>;
    }
    return {};
  }
}
