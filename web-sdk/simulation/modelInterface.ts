import { vehicleDynamicsMB, vehicleDynamicsST, vehicleDynamicsSTD } from '../models/dynamics.js';
import { initMB, initST, initSTD } from '../models/init.js';
import { VehicleParameters } from '../models/types.js';
import { ModelInterface } from './VehicleSimulator.js';
import { ModelType } from './types.js';

export function buildModelInterface(model: ModelType): ModelInterface {
  switch (model) {
    case ModelType.MB:
      return {
        init: (state: number[], params: VehicleParameters) => initMB(state, params),
        dynamics: (x, u, p) => vehicleDynamicsMB(x, u, p),
        speed: (state) => {
          if (state.length > 10) {
            return Math.hypot(state[3], state[10]);
          }
          if (state.length > 3) {
            return Math.abs(state[3]);
          }
          return 0;
        },
      };
    case ModelType.ST:
      return {
        init: (state: number[]) => initST(state),
        dynamics: (x, u, p) => vehicleDynamicsST(x, u, p),
        speed: (state) => (state.length > 3 ? Math.abs(state[3]) : 0),
      };
    case ModelType.STD:
      return {
        init: (state: number[], params: VehicleParameters) => initSTD(state, params),
        dynamics: (x, u, p, dt) => vehicleDynamicsSTD(x, u, p, dt),
        speed: (state) => (state.length > 3 ? Math.abs(state[3]) : 0),
      };
    default:
      throw new Error(`Unsupported model type ${model}`);
  }
}
