import { accelerationConstraints, steeringConstraints } from './constraints.js';
import { formulaLateral, formulaLateralCombined, formulaLongitudinal, formulaLongitudinalCombined } from './tireModel.js';
import { VehicleParameters } from './types.js';
import { vehicleDynamicsKsCog } from './vehicleKinematics.js';

const kGravity = 9.81;

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

export function vehicleDynamicsST(x: number[], uInit: number[], p: VehicleParameters): number[] {
  if (x.length !== 7 || uInit.length !== 2) {
    throw new Error('vehicle_dynamics_st: expected x.size()==7 and uInit.size()==2');
  }

  const mu = p.tire.p_dy1;
  const C_Sf = -p.tire.p_ky1 / p.tire.p_dy1;
  const C_Sr = -p.tire.p_ky1 / p.tire.p_dy1;
  const lf = p.a;
  const lr = p.b;
  const h = p.h_s;
  const m = p.m;
  const I = p.I_z;

  const u = [
    steeringConstraints(x[2], uInit[0], p.steering),
    accelerationConstraints(x[3], uInit[1], p.longitudinal),
  ];

  const f: number[] = [];

  if (Math.abs(x[3]) < 0.1) {
    const lwb = p.a + p.b;
    const xKs = [x[0], x[1], x[2], x[3], x[4]];
    const uArr = [u[0], u[1]];
    const fKs = vehicleDynamicsKsCog(xKs, uArr, p);

    f.push(fKs[0]);
    f.push(fKs[1]);
    f.push(fKs[2]);
    f.push(fKs[3]);
    f.push(fKs[4]);

    const tanDelta = Math.tan(x[2]);
    const cosDelta = Math.cos(x[2]);
    const cosDeltaSq = cosDelta * cosDelta;
    const term = tanDelta * tanDelta * p.b / lwb;

    const d_beta = (p.b * u[0]) / (lwb * cosDeltaSq * (1 + term * term));
    const dd_psi = (1 / lwb) *
      (u[1] * Math.cos(x[6]) * tanDelta -
        x[3] * Math.sin(x[6]) * d_beta * tanDelta +
        x[3] * Math.cos(x[6]) * u[0] / cosDeltaSq);

    f.push(dd_psi);
    f.push(d_beta);
  } else {
    f.push(x[3] * Math.cos(x[6] + x[4]));
    f.push(x[3] * Math.sin(x[6] + x[4]));
    f.push(u[0]);
    f.push(u[1]);

    const term1 = -mu * m / (x[3] * I * (lr + lf)) *
      (lf * lf * C_Sf * (kGravity * lr - u[1] * h) +
        lr * lr * C_Sr * (kGravity * lf + u[1] * h)) * x[5];
    const term2 = mu * m / (I * (lr + lf)) *
      (lr * C_Sr * (kGravity * lf + u[1] * h) -
        lf * C_Sf * (kGravity * lr - u[1] * h)) * x[6];
    const term3 = mu * m / (I * (lr + lf)) *
      lf * C_Sf * (kGravity * lr - u[1] * h) * x[2];

    f.push(x[5]);
    f.push(term1 + term2 + term3);

    const coeff = (mu / (x[3] * x[3] * (lr + lf)) *
      (C_Sr * (kGravity * lf + u[1] * h) * lr -
        C_Sf * (kGravity * lr - u[1] * h) * lf) - 1);
    const c2 = -mu / (x[3] * (lr + lf)) *
      (C_Sr * (kGravity * lf + u[1] * h) +
        C_Sf * (kGravity * lr - u[1] * h));
    const c3 = mu / (x[3] * (lr + lf)) *
      (C_Sf * (kGravity * lr - u[1] * h));

    f.push(coeff * x[5] + c2 * x[6] + c3 * x[2]);
  }

  return f;
}

export function vehicleDynamicsSTD(x: number[], uInit: number[], p: VehicleParameters, dt = 0.01): number[] {
  if (x.length !== 9 || uInit.length !== 2) {
    throw new Error('vehicle_dynamics_std: expected x.size()==9 and u_init.size()==2');
  }

  const lf = p.a;
  const lr = p.b;
  const lwb = p.a + p.b;
  const m = p.m;
  const I = p.I_z;

  const v_s = 0.2;
  const v_b = 0.05;
  const v_min = v_s / 2;

  const u = [
    steeringConstraints(x[2], uInit[0], p.steering),
    accelerationConstraints(x[3], uInit[1], p.longitudinal),
  ];

  let alpha_f = 0;
  let alpha_r = 0;
  if (x[3] > v_min) {
    alpha_f = Math.atan((x[3] * Math.sin(x[6]) + x[5] * lf) / (x[3] * Math.cos(x[6]))) - x[2];
    alpha_r = Math.atan((x[3] * Math.sin(x[6]) - x[5] * lr) / (x[3] * Math.cos(x[6])));
  }

  const F_zf = m * (-u[1] * p.h_s + kGravity * lr) / (lr + lf);
  const F_zr = m * (u[1] * p.h_s + kGravity * lf) / (lr + lf);

  const u_wf = Math.max(
    0,
    x[3] * Math.cos(x[6]) * Math.cos(x[2]) +
    (x[3] * Math.sin(x[6]) + p.a * x[5]) * Math.sin(x[2])
  );
  const u_wr = Math.max(0, x[3] * Math.cos(x[6]));

  const omega_f_state = Math.max(0, x[7]);
  const omega_r_state = Math.max(0, x[8]);

  const s_f = 1 - p.R_w * omega_f_state / Math.max(u_wf, v_min);
  const s_r = 1 - p.R_w * omega_r_state / Math.max(u_wr, v_min);

  const F0_xf = formulaLongitudinal(s_f, 0, F_zf, p.tire);
  const F0_xr = formulaLongitudinal(s_r, 0, F_zr, p.tire);

  const [F0_yf, mu_yf] = formulaLateral(alpha_f, 0, F_zf, p.tire);
  const [F0_yr, mu_yr] = formulaLateral(alpha_r, 0, F_zr, p.tire);

  const F_xf = formulaLongitudinalCombined(s_f, alpha_f, F0_xf, p.tire);
  const F_xr = formulaLongitudinalCombined(s_r, alpha_r, F0_xr, p.tire);

  const F_yf = formulaLateralCombined(s_f, alpha_f, 0, mu_yf, F_zf, F0_yf, p.tire);
  const F_yr = formulaLateralCombined(s_r, alpha_r, 0, mu_yr, F_zr, F0_yr, p.tire);

  let T_B = 0;
  let T_E = 0;
  if (u[1] > 0) {
    T_E = m * p.R_w * u[1];
  } else {
    T_B = m * p.R_w * u[1];
  }

  const d_v = (1 / m) *
    (-F_yf * Math.sin(x[2] - x[6]) +
      F_yr * Math.sin(x[6]) +
      F_xr * Math.cos(x[6]) +
      F_xf * Math.cos(x[2] - x[6]));

  const dd_psi = (1 / I) *
    (F_yf * Math.cos(x[2]) * lf -
      F_yr * lr +
      F_xf * Math.sin(x[2]) * lf);

  let d_beta = 0;
  if (x[3] > v_min) {
    d_beta = -x[5] +
      (1 / (m * x[3])) *
      (F_yf * Math.cos(x[2] - x[6]) +
        F_yr * Math.cos(x[6]) -
        F_xr * Math.sin(x[6]) +
        F_xf * Math.sin(x[2] - x[6]));
  }

  const front_negative = x[7] < 0;
  const rear_negative = x[8] < 0;

  let d_omega_f = 0;
  let d_omega_r = 0;
  if (!front_negative) {
    d_omega_f = (1 / p.I_y_w) *
      (-p.R_w * F_xf + p.T_sb * T_B + p.T_se * T_E);
    if (omega_f_state <= 0 && d_omega_f < 0) {
      d_omega_f = 0;
    }
  }
  if (!rear_negative) {
    d_omega_r = (1 / p.I_y_w) *
      (-p.R_w * F_xr + (1 - p.T_sb) * T_B + (1 - p.T_se) * T_E);
    if (omega_r_state <= 0 && d_omega_r < 0) {
      d_omega_r = 0;
    }
  }

  const xKs = [x[0], x[1], x[2], x[3], x[4]];
  const uArr = [u[0], u[1]];
  const fKs = vehicleDynamicsKsCog(xKs, uArr, p);

  const tanDelta = Math.tan(x[2]);
  const cosDelta = Math.cos(x[2]);
  const cosDeltaSq = cosDelta * cosDelta;
  const term = tanDelta * tanDelta * p.b / lwb;

  const d_beta_ks = (p.b * u[0]) /
    (lwb * cosDeltaSq * (1 + term * term));

  const dd_psi_ks = (1 / lwb) *
    (u[1] * Math.cos(x[6]) * Math.tan(x[2]) -
      x[3] * Math.sin(x[6]) * d_beta_ks * Math.tan(x[2]) +
      x[3] * Math.cos(x[6]) * u[0] / cosDeltaSq);

  const omega_f_clamped = front_negative ? 0 : omega_f_state;
  const omega_r_clamped = rear_negative ? 0 : omega_r_state;
  const inv_tau = 1 / 0.02;
  const d_omega_f_ks = inv_tau * (u_wf / p.R_w - omega_f_clamped);
  const d_omega_r_ks = inv_tau * (u_wr / p.R_w - omega_r_clamped);

  const w_std = 0.5 * (Math.tanh((x[3] - v_s) / v_b) + 1);
  const w_ks = 1 - w_std;

  const f = new Array<number>(9);
  f[0] = x[3] * Math.cos(x[6] + x[4]);
  f[1] = x[3] * Math.sin(x[6] + x[4]);
  f[2] = u[0];
  f[3] = w_std * d_v + w_ks * fKs[3];
  f[4] = w_std * x[5] + w_ks * fKs[4];
  f[5] = w_std * dd_psi + w_ks * dd_psi_ks;
  f[6] = w_std * d_beta + w_ks * d_beta_ks;
  f[7] = w_std * d_omega_f + w_ks * d_omega_f_ks;
  f[8] = w_std * d_omega_r + w_ks * d_omega_r_ks;
  return f;
}

// Ported multi-body model from C++; expects 29-state vector.
export function vehicleDynamicsMB(x: number[], uInit: number[], p: VehicleParameters): number[] {
  if (x.length !== 29 || uInit.length !== 2) {
    throw new Error('vehicle_dynamics_mb: expected x.size()==29 and uInit.size()==2');
  }

  const g = 9.81;

  const u = new Array<number>(2);
  u[0] = steeringConstraints(x[2], uInit[0], p.steering);
  u[1] = accelerationConstraints(x[3], uInit[1], p.longitudinal);

  let beta: number;
  if (Math.abs(x[3]) < 0.1) {
    beta = 0;
  } else {
    beta = Math.atan(x[10] / x[3]);
  }
  const vel = Math.hypot(x[3], x[10]);

  const F_z_LF = (x[16] + p.R_w * (Math.cos(x[13]) - 1) - 0.5 * p.T_f * Math.sin(x[13])) * p.K_zt;
  const F_z_RF = (x[16] + p.R_w * (Math.cos(x[13]) - 1) + 0.5 * p.T_f * Math.sin(x[13])) * p.K_zt;
  const F_z_LR = (x[21] + p.R_w * (Math.cos(x[18]) - 1) - 0.5 * p.T_r * Math.sin(x[18])) * p.K_zt;
  const F_z_RR = (x[21] + p.R_w * (Math.cos(x[18]) - 1) + 0.5 * p.T_r * Math.sin(x[18])) * p.K_zt;

  let u_w_lf = (x[3] + 0.5 * p.T_f * x[5]) * Math.cos(x[2]) +
    (x[10] + p.a * x[5]) * Math.sin(x[2]);
  let u_w_rf = (x[3] - 0.5 * p.T_f * x[5]) * Math.cos(x[2]) +
    (x[10] + p.a * x[5]) * Math.sin(x[2]);
  let u_w_lr = x[3] + 0.5 * p.T_r * x[5];
  let u_w_rr = x[3] - 0.5 * p.T_r * x[5];

  if (u_w_lf < 0) u_w_lf = 0;
  if (u_w_rf < 0) u_w_rf = 0;
  if (u_w_lr < 0) u_w_lr = 0;
  if (u_w_rr < 0) u_w_rr = 0;

  let s_lf: number, s_rf: number, s_lr: number, s_rr: number;
  if (Math.abs(x[3]) < 0.1) {
    s_lf = 0; s_rf = 0; s_lr = 0; s_rr = 0;
  } else {
    const omega_lf = Math.max(0, x[23]);
    const omega_rf = Math.max(0, x[24]);
    const omega_lr = Math.max(0, x[25]);
    const omega_rr = Math.max(0, x[26]);

    const denom_lf = Math.max(u_w_lf, 1e-6);
    const denom_rf = Math.max(u_w_rf, 1e-6);
    const denom_lr = Math.max(u_w_lr, 1e-6);
    const denom_rr = Math.max(u_w_rr, 1e-6);

    s_lf = 1 - p.R_w * omega_lf / denom_lf;
    s_rf = 1 - p.R_w * omega_rf / denom_rf;
    s_lr = 1 - p.R_w * omega_lr / denom_lr;
    s_rr = 1 - p.R_w * omega_rr / denom_rr;
  }

  let alpha_LF: number, alpha_RF: number, alpha_LR: number, alpha_RR: number;
  if (Math.abs(x[3]) < 0.1) {
    alpha_LF = 0; alpha_RF = 0; alpha_LR = 0; alpha_RR = 0;
  } else {
    alpha_LF = Math.atan((x[10] + p.a * x[5] - x[14] * (p.R_w - x[16])) /
      (x[3] + 0.5 * p.T_f * x[5])) - x[2];
    alpha_RF = Math.atan((x[10] + p.a * x[5] - x[14] * (p.R_w - x[16])) /
      (x[3] - 0.5 * p.T_f * x[5])) - x[2];
    alpha_LR = Math.atan((x[10] - p.b * x[5] - x[19] * (p.R_w - x[21])) /
      (x[3] + 0.5 * p.T_r * x[5]));
    alpha_RR = Math.atan((x[10] - p.b * x[5] - x[19] * (p.R_w - x[21])) /
      (x[3] - 0.5 * p.T_r * x[5]));
  }

  const z_SLF = (p.h_s - p.R_w + x[16] - x[11]) / Math.cos(x[6]) - p.h_s + p.R_w +
    p.a * x[8] + 0.5 * (x[6] - x[13]) * p.T_f;
  const z_SRF = (p.h_s - p.R_w + x[16] - x[11]) / Math.cos(x[6]) - p.h_s + p.R_w +
    p.a * x[8] - 0.5 * (x[6] - x[13]) * p.T_f;
  const z_SLR = (p.h_s - p.R_w + x[21] - x[11]) / Math.cos(x[6]) - p.h_s + p.R_w -
    p.b * x[8] + 0.5 * (x[6] - x[18]) * p.T_r;
  const z_SRR = (p.h_s - p.R_w + x[21] - x[11]) / Math.cos(x[6]) - p.h_s + p.R_w -
    p.b * x[8] - 0.5 * (x[6] - x[18]) * p.T_r;

  const dz_SLF = x[17] - x[12] + p.a * x[9] + 0.5 * (x[7] - x[14]) * p.T_f;
  const dz_SRF = x[17] - x[12] + p.a * x[9] - 0.5 * (x[7] - x[14]) * p.T_f;
  const dz_SLR = x[22] - x[12] - p.b * x[9] + 0.5 * (x[7] - x[19]) * p.T_r;
  const dz_SRR = x[22] - x[12] - p.b * x[9] - 0.5 * (x[7] - x[19]) * p.T_r;

  const gamma_LF = x[6] + p.D_f * z_SLF + p.E_f * z_SLF * z_SLF;
  const gamma_RF = x[6] - p.D_f * z_SRF - p.E_f * z_SRF * z_SRF;
  const gamma_LR = x[6] + p.D_r * z_SLR + p.E_r * z_SLR * z_SLR;
  const gamma_RR = x[6] - p.D_r * z_SRR - p.E_r * z_SRR * z_SRR;

  const F0_x_LF = formulaLongitudinal(s_lf, gamma_LF, F_z_LF, p.tire);
  const F0_x_RF = formulaLongitudinal(s_rf, gamma_RF, F_z_RF, p.tire);
  const F0_x_LR = formulaLongitudinal(s_lr, gamma_LR, F_z_LR, p.tire);
  const F0_x_RR = formulaLongitudinal(s_rr, gamma_RR, F_z_RR, p.tire);

  const latLF = formulaLateral(alpha_LF, gamma_LF, F_z_LF, p.tire);
  const F0_y_LF = latLF[0];
  const mu_y_LF = latLF[1];
  const latRF = formulaLateral(alpha_RF, gamma_RF, F_z_RF, p.tire);
  const F0_y_RF = latRF[0];
  const mu_y_RF = latRF[1];
  const latLR = formulaLateral(alpha_LR, gamma_LR, F_z_LR, p.tire);
  const F0_y_LR = latLR[0];
  const mu_y_LR = latLR[1];
  const latRR = formulaLateral(alpha_RR, gamma_RR, F_z_RR, p.tire);
  const F0_y_RR = latRR[0];
  const mu_y_RR = latRR[1];

  const F_x_LF = formulaLongitudinalCombined(s_lf, alpha_LF, F0_x_LF, p.tire);
  const F_x_RF = formulaLongitudinalCombined(s_rf, alpha_RF, F0_x_RF, p.tire);
  const F_x_LR = formulaLongitudinalCombined(s_lr, alpha_LR, F0_x_LR, p.tire);
  const F_x_RR = formulaLongitudinalCombined(s_rr, alpha_RR, F0_x_RR, p.tire);

  const F_y_LF = formulaLateralCombined(s_lf, alpha_LF, gamma_LF, mu_y_LF, F_z_LF, F0_y_LF, p.tire);
  const F_y_RF = formulaLateralCombined(s_rf, alpha_RF, gamma_RF, mu_y_RF, F_z_RF, F0_y_RF, p.tire);
  const F_y_LR = formulaLateralCombined(s_lr, alpha_LR, gamma_LR, mu_y_LR, F_z_LR, F0_y_LR, p.tire);
  const F_y_RR = formulaLateralCombined(s_rr, alpha_RR, gamma_RR, mu_y_RR, F_z_RR, F0_y_RR, p.tire);

  const delta_z_f = p.h_s - p.R_w + x[16] - x[11];
  const delta_z_r = p.h_s - p.R_w + x[21] - x[11];

  const delta_phi_f = x[6] - x[13];
  const delta_phi_r = x[6] - x[18];

  const dot_delta_phi_f = x[7] - x[14];
  const dot_delta_phi_r = x[7] - x[19];

  const dot_delta_z_f = x[17] - x[12];
  const dot_delta_z_r = x[22] - x[12];

  const dot_delta_y_f = x[10] + p.a * x[5] - x[15];
  const dot_delta_y_r = x[10] - p.b * x[5] - x[20];

  const delta_f = delta_z_f * Math.sin(x[6]) - x[27] * Math.cos(x[6]) -
    (p.h_raf - p.R_w) * Math.sin(delta_phi_f);
  const delta_r = delta_z_r * Math.sin(x[6]) - x[28] * Math.cos(x[6]) -
    (p.h_rar - p.R_w) * Math.sin(delta_phi_r);

  const dot_delta_f = (delta_z_f * Math.cos(x[6]) + x[27] * Math.sin(x[6])) * x[7] +
    dot_delta_z_f * Math.sin(x[6]) - dot_delta_y_f * Math.cos(x[6]) -
    (p.h_raf - p.R_w) * Math.cos(delta_phi_f) * dot_delta_phi_f;
  const dot_delta_r = (delta_z_r * Math.cos(x[6]) + x[28] * Math.sin(x[6])) * x[7] +
    dot_delta_z_r * Math.sin(x[6]) - dot_delta_y_r * Math.cos(x[6]) -
    (p.h_rar - p.R_w) * Math.cos(delta_phi_r) * dot_delta_phi_r;

  const s_tf = (p.h_s - p.R_w + x[16] - x[11]) / Math.cos(x[6]);
  const s_tr = (p.h_s - p.R_w + x[21] - x[11]) / Math.cos(x[6]);

  const F_csf = -2 * p.K_zt * (p.D_f * s_tf + 2 * p.E_f * s_tf * s_tf) *
    (x[15] + (0.5 * p.T_f + p.K_lt * (F_y_LF + F_y_RF)) * x[7]);
  const F_csr = -2 * p.K_zt * (p.D_r * s_tr + 2 * p.E_r * s_tr * s_tr) *
    (x[20] - (0.5 * p.T_r + p.K_lt * (F_y_LR + F_y_RR)) * x[7]);

  const z_raf = p.h_raf - p.R_w - delta_f;
  const z_rar = p.h_rar - p.R_w - delta_r;

  const F_z_raf = p.K_ras * z_raf + p.K_tsf * x[27] + p.K_rad * dot_delta_f + F_csf;
  const F_z_rar = p.K_ras * z_rar + p.K_tsr * x[28] + p.K_rad * dot_delta_r + F_csr;

  const beta_dot = (Math.sin(x[6]) * x[3] <= p.h_s * x[7]) ? 0 : x[7];
  const d_vel = (Math.cos(x[6]) * x[3] <= p.h_s * x[7]) ? 0 : x[3];

  const v_sx = x[3];
  const v_sy = x[10];
  const v_sz = x[12];
  const p_sx = x[9];
  const p_sy = x[7];
  const p_sz = x[8];

  const r_sx = -x[20] + x[10] - p.b * x[5];
  const r_sy = x[8];
  const r_sz = x[22] - x[12];
  const p_ux = -x[19];
  const p_uy = x[5];
  const p_uz = x[9];

  const sum_F_ltf = F_y_LF + F_y_RF;
  const sum_F_ltr = F_y_LR + F_y_RR;
  const sum_F_xtf = F_x_LF + F_x_RF;
  const sum_F_xtr = F_x_LR + F_x_RR;

  const I_y_f = p.I_uf + p.m_uf * (p.h_s - p.R_w + x[16]) * (p.h_s - p.R_w + x[16]);
  const I_y_r = p.I_ur + p.m_ur * (p.h_s - p.R_w + x[21]) * (p.h_s - p.R_w + x[21]);

  const A_11 = p.m_s + 2 * p.m_uf + 2 * p.m_ur;
  const A_12 = -(2 * p.m_uf + 2 * p.m_ur) * x[7];
  const A_13 = (p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[9] + (2 * p.m_uf + 2 * p.m_ur) * v_sy;

  const B_11 = -sum_F_xtf - sum_F_xtr - (p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[8];
  const B_12 = -(sum_F_ltf + sum_F_ltr) * x[7] - (p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[6];
  const B_13 = -(sum_F_ltf + sum_F_ltr) * x[9] + (2 * p.m_uf + 2 * p.m_ur) * (x[8] * x[7] + x[9] * x[6]) +
    (p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[10];

  const A_21 = (2 * p.m_uf + 2 * p.m_ur) * x[7];
  const A_22 = p.m_s + 2 * p.m_uf + 2 * p.m_ur;
  const A_23 = -(p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[7] - (2 * p.m_uf + 2 * p.m_ur) * v_sx;

  const B_21 = sum_F_ltf + sum_F_ltr + (p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[6];
  const B_22 = -sum_F_xtf * x[7] - sum_F_xtr * x[7] + (p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[8];
  const B_23 = sum_F_xtf * x[9] + sum_F_xtr * x[9] - (2 * p.m_uf + 2 * p.m_ur) * (x[7] * x[8] + x[6] * x[9]) - (p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[3];

  const A_31 = -(p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[9];
  const A_32 = (p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[7];
  const A_33 = p.I_y_s + 2 * p.m_uf * x[16] * x[16] + 2 * p.m_ur * x[21] * x[21] + (2 * p.m_uf + 2 * p.m_ur) * v_sx * v_sx;

  const B_31 = (p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[8] +
    (p.m_s * x[12] + p.m_uf * x[17] + p.m_uf * x[17] + p.m_ur * x[22] + p.m_ur * x[22]) * g;
  const B_32 = (p.m_s * x[12] + p.m_uf * x[17] + p.m_uf * x[17] + p.m_ur * x[22] + p.m_ur * x[22]) * g * x[7] +
    (p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[3];
  const B_33 = -(p.m_s * p.h_s - 2 * p.m_uf * x[16] - 2 * p.m_ur * x[21]) * x[10] +
    (2 * p.m_uf + 2 * p.m_ur) * x[3] * x[8] + (p.m_s * x[12] + 2 * p.m_uf * x[17] + 2 * p.m_ur * x[22]) * x[6] * g;

  const detA = A_11 * (A_22 * A_33 - A_23 * A_32) - A_12 * (A_21 * A_33 - A_23 * A_31) + A_13 * (A_21 * A_32 - A_22 * A_31);
  const d_velx = ((B_11 * (A_22 * A_33 - A_23 * A_32) - A_12 * (B_21 * A_33 - A_23 * B_31) + A_13 * (B_21 * A_32 - A_22 * B_31)) / detA) + u[1];
  const d_vely = (A_11 * (B_21 * A_33 - A_23 * B_31) - B_11 * (A_21 * A_33 - A_23 * A_31) + A_13 * (A_21 * B_31 - B_21 * A_31)) / detA;
  const d_omega = (A_11 * (A_22 * B_31 - B_21 * A_32) - A_12 * (A_21 * B_31 - B_21 * A_31) + B_11 * (A_21 * A_32 - A_22 * A_31)) / detA;

  const sum_F_xt = sum_F_xtf + sum_F_xtr;
  const sum_F_yt = sum_F_ltf + sum_F_ltr;
  const d_beta_cg = (sum_F_yt * Math.cos(beta) - sum_F_xt * Math.sin(beta) - p.m * x[3] * x[5]) / (p.m * vel);

  const omega_lf = x[23];
  const omega_rf = x[24];
  const omega_lr = x[25];
  const omega_rr = x[26];

  const wheelbase = p.a + p.b;
  const l_wb = wheelbase > 0 ? wheelbase : 1;

  const dd_psif = (x[9] * Math.cos(beta) * Math.tan(x[2]) - x[10] * x[9] * Math.sin(beta) * Math.tan(x[2]) + (x[10] * Math.cos(beta) * Math.cos(beta)) * u[0]) / l_wb;

  const min_vel = 0.01;
  const dd_psir = (p.R_w * (F_y_LF + F_y_RF + F_y_LR + F_y_RR) + p.m * x[3] * x[5]) /
    (vel <= min_vel ? min_vel : vel);

  let dd_psi = clamp(
    (vel - 0.4) / (vel + 0.1),
    0,
    1
  );
  dd_psi = dd_psi * dd_psir + (1 - dd_psi) * dd_psif;

  const F_L = p.T_se * p.m * u[1] * Math.sin(Math.abs(x[2])) / p.l;

  const B = p.K_zt * p.K_lt;

  const d_omega_lf = (1 / p.I_y_w) * (-p.R_w * F_x_LF - 0.5 * p.R_w * F_L + p.T_sb * p.T_se * p.m * u[1] * p.R_w + u[0] * p.m_s * p.h_s + F_L * p.R_w * p.K_lt * B * x[14]);
  const d_omega_rf = (1 / p.I_y_w) * (-p.R_w * F_x_RF - 0.5 * p.R_w * F_L + p.T_sb * p.T_se * p.m * u[1] * p.R_w - u[0] * p.m_s * p.h_s - F_L * p.R_w * p.K_lt * B * x[14]);
  const d_omega_lr = (1 / p.I_y_w) * (-p.R_w * F_x_LR - 0.5 * p.R_w * F_L + (1 - p.T_sb) * p.T_se * p.m * u[1] * p.R_w + F_L * p.R_w * p.K_lt * B * x[19]);
  const d_omega_rr = (1 / p.I_y_w) * (-p.R_w * F_x_RR - 0.5 * p.R_w * F_L + (1 - p.T_sb) * p.T_se * p.m * u[1] * p.R_w - F_L * p.R_w * p.K_lt * B * x[19]);

  const d_delta_yf = (p.h_raf - p.R_w) * x[5] - x[10];
  const d_delta_yr = (p.h_rar - p.R_w) * x[5] - x[10];

  const dxf = (d_vel * Math.cos(x[5]) + x[3] * x[5] * Math.sin(x[5])) * Math.cos(beta) - vel * d_beta_cg * Math.sin(beta);
  const dyf = (d_vel * Math.sin(x[5]) - x[3] * x[5] * Math.cos(x[5])) * Math.cos(beta) + vel * d_beta_cg * Math.cos(beta);

  const f: number[] = [];
  f.push(dxf);
  f.push(dyf);
  f.push(u[0]);
  f.push(d_velx);
  f.push(x[5]);
  f.push(dd_psi);
  f.push(beta_dot);
  f.push(x[14]);
  f.push(x[27]);
  f.push(x[9]);
  f.push(d_vely);
  f.push(x[12]);
  f.push(d_omega);
  f.push(x[15]);
  f.push(d_omega_lf);
  f.push(d_delta_yf);
  f.push(x[17]);
  f.push((F_z_LF + F_z_RF - p.m_uf * g - p.m_uf * p.h_raf * dd_psi - p.m_uf * p.h_raf * x[9] * x[9]) / (2 * p.m_uf));
  f.push(x[19]);
  f.push(d_omega_lr);
  f.push(d_delta_yr);
  f.push(x[22]);
  f.push((F_z_LR + F_z_RR - p.m_ur * g - p.m_ur * p.h_rar * dd_psi - p.m_ur * p.h_rar * x[9] * x[9]) / (2 * p.m_ur));
  f.push(d_omega_rf);
  f.push(d_omega_rr);
  f.push(x[27]);
  f.push(d_delta_f);
  f.push(x[28]);
  f.push(d_delta_r);

  return f;
}
