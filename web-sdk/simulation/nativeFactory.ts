import type { NativeDaemonFactory } from './backend.js';

/**
 * Optional packaged native daemon factory. When unavailable, HybridSimulationBackend
 * transparently falls back to the pure JavaScript implementations.
 */
export const packagedNativeFactory: NativeDaemonFactory | undefined = undefined;

export default packagedNativeFactory;
