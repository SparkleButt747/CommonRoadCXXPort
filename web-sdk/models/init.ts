import { VehicleParameters } from './types.js';

export function initST(init: number[]): number[] {
  const size = 7;
  const state = new Array<number>(size).fill(0);
  const copy = Math.min(init.length, size);
  for (let i = 0; i < copy; i += 1) {
    state[i] = init[i] ?? 0;
  }
  return state;
}

export function initSTD(init: number[], params: VehicleParameters): number[] {
  const baseSize = 7;
  const x0: number[] = new Array(baseSize).fill(0);
  const copy = Math.min(init.length, baseSize);
  for (let i = 0; i < copy; i += 1) {
    x0[i] = init[i] ?? 0;
  }

  const v = x0[3];
  const beta = x0[6];
  const delta = x0[2];

  x0.push(v * Math.cos(beta) * Math.cos(delta) / Math.max(params.R_w, 1e-9));
  x0.push(v * Math.cos(beta) / Math.max(params.R_w, 1e-9));
  return x0;
}

export function initMB(init: number[], params: VehicleParameters): number[] {
  const baseSize = 7;
  const base: number[] = new Array(baseSize).fill(0);
  const copy = Math.min(init.length, base.length);
  for (let i = 0; i < copy; i += 1) {
    base[i] = init[i] ?? 0;
  }

  const sx0 = base[0];
  const sy0 = base[1];
  const delta0 = base[2];
  const vel0 = base[3];
  const psi0 = base[4];
  const dotPsi0 = base[5];
  const beta0 = base[6];

  const g = 9.81;
  const F0_z_f = params.m_s * g * params.b / (params.a + params.b) + params.m_uf * g;
  const F0_z_r = params.m_s * g * params.a / (params.a + params.b) + params.m_ur * g;

  const x0: number[] = [];
  x0.push(sx0);
  x0.push(sy0);
  x0.push(delta0);
  x0.push(Math.cos(beta0) * vel0);
  x0.push(psi0);
  x0.push(dotPsi0);
  x0.push(0);
  x0.push(0);
  x0.push(0);
  x0.push(0);
  x0.push(Math.sin(beta0) * vel0);
  x0.push(0);
  x0.push(0);

  x0.push(0);
  x0.push(0);
  x0.push(Math.sin(beta0) * vel0 + params.a * dotPsi0);
  x0.push(F0_z_f / (2 * params.K_zt));
  x0.push(0);

  x0.push(0);
  x0.push(0);
  x0.push(Math.sin(beta0) * vel0 - params.b * dotPsi0);
  x0.push(F0_z_r / (2 * params.K_zt));
  x0.push(0);

  const omega0 = x0[3] / Math.max(params.R_w, 1e-9);
  x0.push(omega0);
  x0.push(omega0);
  x0.push(omega0);
  x0.push(omega0);

  x0.push(0);
  x0.push(0);
  return x0;
}
