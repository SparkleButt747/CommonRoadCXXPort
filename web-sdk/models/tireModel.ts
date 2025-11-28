import { TireParameters } from './types.js';

function sign(value: number): number {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

export function formulaLongitudinal(kappa: number, gamma: number, Fz: number, p: TireParameters): number {
  let slip = -kappa;
  const S_hx = p.p_hx1;
  const S_vx = Fz * p.p_vx1;

  const kappaX = slip + S_hx;
  const muX = p.p_dx1 * (1 - p.p_dx3 * gamma * gamma);

  const Cx = p.p_cx1;
  const Dx = muX * Fz;
  const Ex = p.p_ex1;
  const Kx = Fz * p.p_kx1;
  const Bx = Dx !== 0 ? Kx / (Cx * Dx) : 0;

  return Dx * Math.sin(Cx * Math.atan(Bx * kappaX - Ex * (Bx * kappaX - Math.atan(Bx * kappaX))) + S_vx);
}

export function formulaLateral(alpha: number, gamma: number, Fz: number, p: TireParameters): [number, number] {
  const S_hy = sign(gamma) * (p.p_hy1 + p.p_hy3 * Math.abs(gamma));
  const S_vy = sign(gamma) * Fz * (p.p_vy1 + p.p_vy3 * Math.abs(gamma));

  const alphaY = alpha + S_hy;
  const muY = p.p_dy1 * (1 - p.p_dy3 * gamma * gamma);

  const Cy = p.p_cy1;
  const Dy = muY * Fz;
  const Ey = p.p_ey1;
  const Ky = Fz * p.p_ky1;
  const By = Dy !== 0 ? Ky / (Cy * Dy) : 0;

  const Fy = Dy * Math.sin(
    Cy * Math.atan(By * alphaY - Ey * (By * alphaY - Math.atan(By * alphaY)))
  ) + S_vy;

  return [Fy, muY];
}

export function formulaLongitudinalCombined(kappa: number, alpha: number, F0x: number, p: TireParameters): number {
  const S_hxalpha = p.r_hx1;
  const alphaS = alpha + S_hxalpha;

  const B_xalpha = p.r_bx1 * Math.cos(Math.atan(p.r_bx2 * kappa));
  const C_xalpha = p.r_cx1;
  const E_xalpha = p.r_ex1;

  const denom = C_xalpha * Math.atan(
    B_xalpha * S_hxalpha - E_xalpha * (B_xalpha * S_hxalpha - Math.atan(B_xalpha * S_hxalpha))
  );
  const D_xalpha = Math.cos(denom) !== 0 ? F0x / Math.cos(denom) : 0;

  return D_xalpha * Math.cos(
    C_xalpha * Math.atan(B_xalpha * alphaS - E_xalpha * (B_xalpha * alphaS - Math.atan(B_xalpha * alphaS)))
  );
}

export function formulaLateralCombined(
  kappa: number,
  alpha: number,
  gamma: number,
  muY: number,
  Fz: number,
  F0y: number,
  p: TireParameters
): number {
  const S_hykappa = p.r_hy1;
  const kappaS = kappa + S_hykappa;

  const B_ykappa = p.r_by1 * Math.cos(Math.atan(p.r_by2 * (alpha - p.r_by3)));
  const C_ykappa = p.r_cy1;
  const E_ykappa = p.r_ey1;

  const denom = C_ykappa * Math.atan(
    B_ykappa * S_hykappa - E_ykappa * (B_ykappa * S_hykappa - Math.atan(B_ykappa * S_hykappa))
  );
  const D_ykappa = Math.cos(denom) !== 0 ? F0y / Math.cos(denom) : 0;

  const D_vykappa = muY * Fz * (p.r_vy1 + p.r_vy3 * gamma) * Math.cos(Math.atan(p.r_vy4 * alpha));
  const S_vykappa = D_vykappa * Math.sin(p.r_vy5 * Math.atan(p.r_vy6 * kappa));

  return D_ykappa * Math.cos(
    C_ykappa * Math.atan(B_ykappa * kappaS - E_ykappa * (B_ykappa * kappaS - Math.atan(B_ykappa * kappaS)))
  ) + S_vykappa;
}
