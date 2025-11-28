// Core parameter and model types mirroring the C++ velox structures.
export interface SteeringParameters {
  min: number;
  max: number;
  v_min: number;
  v_max: number;
  kappa_dot_max: number;
  kappa_dot_dot_max: number;
}

export interface LongitudinalParameters {
  v_min: number;
  v_max: number;
  v_switch: number;
  a_max: number;
  j_max: number;
  j_dot_max: number;
}

export interface TrailerParameters {
  l: number;
  w: number;
  l_hitch: number;
  l_total: number;
  l_wb: number;
}

export interface TireParameters {
  // Longitudinal coefficients
  p_cx1: number;
  p_dx1: number;
  p_dx3: number;
  p_ex1: number;
  p_kx1: number;
  p_hx1: number;
  p_vx1: number;
  r_bx1: number;
  r_bx2: number;
  r_cx1: number;
  r_ex1: number;
  r_hx1: number;

  // Lateral coefficients
  p_cy1: number;
  p_dy1: number;
  p_dy3: number;
  p_ey1: number;
  p_ky1: number;
  p_hy1: number;
  p_hy3: number;
  p_vy1: number;
  p_vy3: number;
  r_by1: number;
  r_by2: number;
  r_by3: number;
  r_cy1: number;
  r_ey1: number;
  r_hy1: number;
  r_vy1: number;
  r_vy3: number;
  r_vy4: number;
  r_vy5: number;
  r_vy6: number;
}

export interface VehicleParameters {
  // vehicle body dimensions
  l: number;
  w: number;

  // steering parameters
  steering: SteeringParameters;

  // longitudinal parameters
  longitudinal: LongitudinalParameters;

  // masses
  m: number;
  m_s: number;
  m_uf: number;
  m_ur: number;

  // axes distances
  a: number;
  b: number;

  // moments of inertia of sprung mass
  I_Phi_s: number;
  I_y_s: number;
  I_z: number;
  I_xz_s: number;

  // suspension parameters
  K_sf: number;
  K_sdf: number;
  K_sr: number;
  K_sdr: number;

  // geometric parameters
  T_f: number;
  T_r: number;
  K_ras: number;

  K_tsf: number;
  K_tsr: number;
  K_rad: number;
  K_zt: number;

  h_cg: number;
  h_raf: number;
  h_rar: number;

  h_s: number;

  I_uf: number;
  I_ur: number;
  I_y_w: number;

  K_lt: number;
  R_w: number;

  // split of brake and engine torque
  T_sb: number;
  T_se: number;

  // suspension camber parameters
  D_f: number;
  D_r: number;
  E_f: number;
  E_r: number;

  // tire parameters
  tire: TireParameters;

  // trailer parameters (used for kst)
  trailer: TrailerParameters;
}

export type NumericRecord = Record<string, number>;
