import type { NativeDaemonFactory } from './backend.js';

/**
 * The packaged native daemon must be provided by real native bindings (e.g.,
 * WASM or an embedded binary). The previous kinematic stub diverged from the
 * native MB/ST/STD solvers and could mask missing dependencies, so this
 * factory now fails fast until a concrete implementation is wired in.
 */
export const packagedNativeFactory: NativeDaemonFactory = async () => {
  throw new Error(
    'Packaged native daemon unavailable: provide a NativeDaemonFactory backed by MB/ST/STD bindings (native or WASM).'
  );
};

export default packagedNativeFactory;
