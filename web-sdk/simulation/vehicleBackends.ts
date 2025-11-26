import { JsSimulationBackend, JsBackendOptions } from './jsBackend.js';
import { ModelType } from './types.js';

export type VehicleBackendOptions = Omit<JsBackendOptions, 'model'> & { model?: ModelType };

export class MBSimulationBackend extends JsSimulationBackend {
  constructor(options: VehicleBackendOptions) {
    super({ ...options, model: ModelType.MB });
  }
}

export class STSimulationBackend extends JsSimulationBackend {
  constructor(options: VehicleBackendOptions) {
    super({ ...options, model: ModelType.ST });
  }
}

export class STDSimulationBackend extends JsSimulationBackend {
  constructor(options: VehicleBackendOptions) {
    super({ ...options, model: ModelType.STD });
  }
}
