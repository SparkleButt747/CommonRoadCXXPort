export interface RollingResistanceConfig {
  c_rr: number;
}

export class RollingResistance {
  private readonly cfg: RollingResistanceConfig;
  private readonly gravity: number;

  constructor(config: RollingResistanceConfig, gravity = 9.81) {
    this.validate(config, gravity);
    this.cfg = config;
    this.gravity = gravity;
  }

  force(speed: number, normalForce: number): number {
    const base = this.cfg.c_rr * Math.max(0, normalForce);
    if (speed === 0) return 0;
    return -Math.sign(speed) * base;
  }

  private validate(config: RollingResistanceConfig, gravity: number): void {
    if (!Number.isFinite(config.c_rr) || config.c_rr < 0) {
      throw new Error('rolling_resistance.c_rr must be finite and non-negative');
    }
    if (!Number.isFinite(gravity) || gravity <= 0) {
      throw new Error('gravity must be positive and finite');
    }
  }
}
