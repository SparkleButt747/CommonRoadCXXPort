import { LowSpeedSafety } from './LowSpeedSafety.js';
import { VehicleParameters } from '../models/types.js';

export interface ModelInterface {
  init: (state: number[], params: VehicleParameters) => number[];
  dynamics: (state: number[], control: number[], params: VehicleParameters, dt: number) => number[];
  speed: (state: number[], params: VehicleParameters) => number;
}

export class VehicleSimulator {
  private state: number[] = [];
  private ready = false;

  constructor(
    private model: ModelInterface,
    private params: VehicleParameters,
    private dt: number,
    private safety: LowSpeedSafety
  ) {
    if (!this.model || !this.model.init || !this.model.dynamics || !this.model.speed) {
      throw new Error('VehicleSimulator requires a valid ModelInterface');
    }
    this.setDt(dt);
    this.safety.reset();
  }

  reset(initial: number[]): void {
    this.safety.reset();
    this.state = this.model.init(initial, this.params);
    this.applySafety(this.state, true);
    this.ready = true;
  }

  setDt(dt: number): void {
    if (!(dt > 0)) {
      throw new Error('VehicleSimulator timestep must be positive');
    }
    this.dt = dt;
  }

  dtSeconds(): number {
    return this.dt;
  }

  speed(): number {
    this.ensureReady();
    return this.model.speed(this.state, this.params);
  }

  currentState(): number[] {
    this.ensureReady();
    return this.state;
  }

  step(control: number[]): number[] {
    this.ensureReady();
    if (control.length !== 2) {
      throw new Error('VehicleSimulator control must contain steering rate and acceleration');
    }
    const dt = this.dt;
    const previous = this.state.slice();

    const [k1, state1] = this.dynamics(previous, control, true);
    this.state = state1;

    const k2State = this.addScaled(this.state, 0.5 * dt, k1);
    const [k2] = this.dynamics(k2State, control, false);

    const k3State = this.addScaled(this.state, 0.5 * dt, k2);
    const [k3] = this.dynamics(k3State, control, false);

    const k4State = this.addScaled(this.state, dt, k3);
    const [k4] = this.dynamics(k4State, control, false);

    if ([k1, k2, k3, k4].some((v) => v.length !== this.state.length)) {
      throw new Error('VehicleSimulator dynamics returned mismatched state dimension');
    }

    for (let i = 0; i < this.state.length; i += 1) {
      this.state[i] += (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }

    const longIndex = this.safety.longitudinalIndex();
    if (longIndex !== undefined) {
      const idx = longIndex;
      if (idx < previous.length && idx < this.state.length) {
        const prevLong = previous[idx];
        const currLong = this.state[idx];
        if (prevLong >= 0 && currLong < 0) {
          this.state[idx] = 0;
        }
      }
    }

    this.applySafety(this.state, true);
    return this.state;
  }

  safetySystem(): LowSpeedSafety {
    return this.safety;
  }

  private ensureReady(): void {
    if (!this.ready) {
      throw new Error('VehicleSimulator has not been initialised; call reset() first');
    }
  }

  private addScaled(base: number[], scale: number, delta: number[]): number[] {
    if (base.length !== delta.length) {
      throw new Error('VehicleSimulator::add_scaled size mismatch');
    }
    const result = new Array<number>(base.length);
    for (let i = 0; i < base.length; i += 1) {
      result[i] = base[i] + scale * delta[i];
    }
    return result;
  }

  private dynamics(state: number[], control: number[], updateLatch: boolean): [number[], number[]] {
    const sanitized = state.slice();
    this.applySafety(sanitized, updateLatch);
    const rhs = this.model.dynamics(sanitized, control, this.params, this.dt);
    return [rhs, sanitized];
  }

  private applySafety(state: number[], updateLatch: boolean): void {
    const speed = this.model.speed(state, this.params);
    this.safety.apply(state, speed, updateLatch);
  }
}
