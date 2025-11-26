#!/usr/bin/env node
import { readFile } from 'fs/promises';
import { access } from 'fs';
import path from 'path';
import { fileURLToPath, pathToFileURL } from 'url';
import { HybridSimulationBackend, NativeDaemonFactory } from '../simulation/backend.js';
import packagedNativeFactory from '../simulation/nativeFactory.js';
import { ConfigManager } from '../io/ConfigManager.js';
import { ModelType } from '../simulation/types.js';
import type { BackendSnapshot, SimulationBackend } from '../simulation/backend.js';
import type { SimulationTelemetry } from '../telemetry/index.js';

interface TraceSegment {
  steps: number;
  steerRate?: number;
  accel?: number;
  dt?: number;
}

interface ScenarioTolerance {
  default?: number;
  fields?: Record<string, number>;
}

interface ScenarioSpec {
  name: string;
  model: keyof typeof ModelType | ModelType;
  vehicleId: number;
  initialState: number[];
  dt: number;
  driftEnabled?: boolean;
  trace: TraceSegment[];
  tolerances?: ScenarioTolerance;
}

interface CliOptions {
  fixturePath: string;
  nativeFactoryPath?: string;
  parameterRoot: string;
  configRoot?: string;
  toleranceDefault?: number;
}

interface FieldStats {
  count: number;
  rmse: number;
  maxAbs: number;
}

interface ComparisonResult {
  scenario: string;
  samples: number;
  fields: Map<string, FieldStats>;
  overallRmse: number;
  overallMax: number;
  toleranceBreaches: Array<{ field: string; maxAbs: number; tolerance: number }>;
}

function usage(): never {
  console.error(`Usage: compareNative --fixture <path> --native <module[:export]> [--parameterRoot <path>] [--configRoot <path>] [--tolerance <value>]`);
  process.exit(1);
}

function parseArgs(argv: string[]): CliOptions {
  const options: CliOptions = {
    fixturePath: path.resolve('web-sdk/tools/fixtures/driveTraces.json'),
    parameterRoot: path.resolve('parameters'),
  };

  for (let i = 2; i < argv.length; i++) {
    const arg = argv[i];
    if (arg === '--fixture') {
      options.fixturePath = path.resolve(argv[++i] ?? '');
    } else if (arg === '--native') {
      options.nativeFactoryPath = argv[++i];
    } else if (arg === '--parameterRoot') {
      options.parameterRoot = path.resolve(argv[++i] ?? '');
    } else if (arg === '--configRoot') {
      options.configRoot = path.resolve(argv[++i] ?? '');
    } else if (arg === '--tolerance') {
      options.toleranceDefault = Number(argv[++i]);
    } else if (arg === '--help' || arg === '-h') {
      usage();
    }
  }

  return options;
}

async function loadScenarios(fixturePath: string): Promise<ScenarioSpec[]> {
  const raw = await readFile(fixturePath, 'utf8');
  const parsed = JSON.parse(raw);
  const scenarios = parsed.scenarios as ScenarioSpec[];
  if (!Array.isArray(scenarios) || scenarios.length === 0) {
    throw new Error(`No scenarios found in ${fixturePath}`);
  }
  return scenarios;
}

function toModel(value: keyof typeof ModelType | ModelType): ModelType {
  if (typeof value === 'string') {
    const key = value as keyof typeof ModelType;
    if (ModelType[key]) {
      return ModelType[key];
    }
  }
  return value as ModelType;
}

function buildFileFetcher(root: string) {
  const normalizedRoot = path.resolve(root);
  return async function fetcher(input: RequestInfo | URL, init?: RequestInit): Promise<Response> {
    const raw = typeof input === 'string' ? input : input.toString();
    const url = raw.startsWith('file:') ? raw : pathToFileURL(path.isAbsolute(raw) ? raw : path.join(normalizedRoot, raw)).toString();
    const filePath = fileURLToPath(url);
    const method = (init?.method ?? 'GET').toUpperCase();
    await new Promise<void>((resolve, reject) => access(filePath, (err) => (err ? reject(err) : resolve())));
    if (method === 'HEAD') {
      return new Response('', { status: 200 });
    }
    const contents = await readFile(filePath);
    return new Response(contents, { status: 200 });
  };
}

function fileRoot(root: string): string {
  return pathToFileURL(path.resolve(root)).toString() + (root.endsWith('/') ? '' : '/');
}

async function loadNativeFactory(moduleSpec: string): Promise<NativeDaemonFactory> {
  const [modulePath, exportName] = moduleSpec.split(':');
  const mod = await import(path.isAbsolute(modulePath) ? pathToFileURL(modulePath).href : pathToFileURL(path.resolve(modulePath)).href);
  const factory = (exportName ? mod[exportName] : mod.default) as NativeDaemonFactory | undefined;
  if (typeof factory !== 'function') {
    throw new Error(`Module ${moduleSpec} does not export a NativeDaemonFactory`);
  }
  return factory;
}

async function buildBackend(
  model: ModelType,
  vehicleId: number,
  driftEnabled: boolean,
  configManager: ConfigManager,
  nativeFactory?: NativeDaemonFactory
): Promise<SimulationBackend> {
  const backend = new HybridSimulationBackend({
    model,
    vehicleId,
    configManager,
    nativeFactory,
    driftEnabled,
  });
  if (backend.ready) {
    await backend.ready;
  }
  return backend;
}

async function runTrace(
  backend: SimulationBackend,
  initialState: number[],
  dt: number,
  trace: TraceSegment[]
): Promise<{ snapshots: BackendSnapshot[]; telemetries: SimulationTelemetry[] }> {
  backend.reset(initialState, dt);
  const snapshots: BackendSnapshot[] = [];
  const telemetries: SimulationTelemetry[] = [];
  for (const segment of trace) {
    const steerRate = segment.steerRate ?? 0;
    const accel = segment.accel ?? 0;
    const stepDt = segment.dt ?? dt;
    for (let i = 0; i < segment.steps; i++) {
      backend.step([steerRate, accel], stepDt);
      const snap = backend.snapshot();
      snapshots.push(structuredClone(snap));
      if (snap.telemetry) {
        telemetries.push(structuredClone(snap.telemetry));
      }
    }
  }
  return { snapshots, telemetries };
}

function flattenTelemetry(obj: any, prefix = ''): Record<string, number> {
  const result: Record<string, number> = {};
  const base = prefix ? `${prefix}.` : '';
  for (const [key, value] of Object.entries(obj ?? {})) {
    const next = `${base}${key}`;
    if (value && typeof value === 'object' && !Array.isArray(value)) {
      Object.assign(result, flattenTelemetry(value, next));
    } else if (typeof value === 'number') {
      result[next] = value;
    } else if (typeof value === 'boolean') {
      result[next] = value ? 1 : 0;
    }
  }
  return result;
}

function compareStreams(
  scenario: ScenarioSpec,
  jsTelem: SimulationTelemetry[],
  nativeTelem: SimulationTelemetry[],
  toleranceOverride?: number
): ComparisonResult {
  const minSamples = Math.min(jsTelem.length, nativeTelem.length);
  const fieldStats = new Map<string, FieldStats>();
  const toleranceBreaches: Array<{ field: string; maxAbs: number; tolerance: number }> = [];
  const defaultTol = toleranceOverride ?? scenario.tolerances?.default ?? 1e-3;
  const fieldTol = scenario.tolerances?.fields ?? {};

  let totalSq = 0;
  let totalCount = 0;
  let overallMax = 0;

  for (let i = 0; i < minSamples; i++) {
    const left = flattenTelemetry(jsTelem[i]);
    const right = flattenTelemetry(nativeTelem[i]);
    const keys = new Set([...Object.keys(left), ...Object.keys(right)]);
    for (const key of keys) {
      const l = left[key] ?? 0;
      const r = right[key] ?? 0;
      const diff = r - l;
      const abs = Math.abs(diff);
      const stats = fieldStats.get(key) ?? { count: 0, rmse: 0, maxAbs: 0 };
      stats.count += 1;
      stats.rmse += diff * diff;
      stats.maxAbs = Math.max(stats.maxAbs, abs);
      fieldStats.set(key, stats);
      totalSq += diff * diff;
      totalCount += 1;
      overallMax = Math.max(overallMax, abs);
    }
  }

  for (const [key, stats] of fieldStats.entries()) {
    stats.rmse = Math.sqrt(stats.rmse / Math.max(stats.count, 1));
    const tol = fieldTol[key] ?? defaultTol;
    if (stats.maxAbs > tol) {
      toleranceBreaches.push({ field: key, maxAbs: stats.maxAbs, tolerance: tol });
    }
  }

  const overallRmse = Math.sqrt(totalSq / Math.max(totalCount, 1));

  return {
    scenario: scenario.name,
    samples: minSamples,
    fields: fieldStats,
    overallRmse,
    overallMax,
    toleranceBreaches,
  };
}

function printReport(result: ComparisonResult): void {
  console.log(`\nScenario: ${result.scenario}`);
  console.log(`Samples: ${result.samples}`);
  console.log(`Overall RMSE: ${result.overallRmse.toExponential(6)} | Max abs diff: ${result.overallMax.toExponential(6)}`);
  if (result.toleranceBreaches.length === 0) {
    console.log('All telemetry signals within tolerance.');
    return;
  }
  console.log('Signals outside tolerance:');
  result.toleranceBreaches
    .sort((a, b) => b.maxAbs - a.maxAbs)
    .slice(0, 10)
    .forEach((entry) => {
      console.log(`  ${entry.field}: max diff ${entry.maxAbs.toExponential(6)} (tolerance ${entry.tolerance})`);
    });
}

async function main(): Promise<void> {
  const options = parseArgs(process.argv);
  const scenarios = await loadScenarios(options.fixturePath);
  const parameterRoot = fileRoot(options.parameterRoot);
  const configRoot = fileRoot(options.configRoot ?? path.join(path.dirname(options.parameterRoot), 'config'));
  const fetcher = buildFileFetcher('/');
  const configManager = new ConfigManager(configRoot, parameterRoot, fetcher);
  const nativeFactory = options.nativeFactoryPath
    ? await loadNativeFactory(options.nativeFactoryPath)
    : packagedNativeFactory;

  for (const scenario of scenarios) {
    const model = toModel(scenario.model);
    const jsBackend = await buildBackend(model, scenario.vehicleId, !!scenario.driftEnabled, configManager);
    const nativeBackend = await buildBackend(model, scenario.vehicleId, !!scenario.driftEnabled, configManager, nativeFactory);

    const js = await runTrace(jsBackend, scenario.initialState, scenario.dt, scenario.trace);
    const native = await runTrace(nativeBackend, scenario.initialState, scenario.dt, scenario.trace);

    const result = compareStreams(scenario, js.telemetries, native.telemetries, options.toleranceDefault);
    printReport(result);
  }
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
