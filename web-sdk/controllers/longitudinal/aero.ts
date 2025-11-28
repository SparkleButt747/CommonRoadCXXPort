export interface AeroConfig {
  drag_coefficient: number;
  downforce_coefficient: number;
}

export class AeroModel {
  private readonly cfg: AeroConfig;

  constructor(config: AeroConfig) {
    this.validate(config);
    this.cfg = config;
  }

  drag_force(speed: number): number {
    if (speed === 0) return 0;
    const coeff = this.cfg.drag_coefficient;
    const magnitude = coeff * speed * speed;
    return -Math.sign(speed) * magnitude;
  }

  downforce(speed: number): number {
    if (this.cfg.downforce_coefficient === 0 || speed === 0) return 0;
    const coeff = Math.abs(this.cfg.downforce_coefficient);
    return coeff * speed * speed;
  }

  private validate(cfg: AeroConfig): void {
    if (!Number.isFinite(cfg.drag_coefficient) || !Number.isFinite(cfg.downforce_coefficient)) {
      throw new Error('aero config must be finite');
    }
    if (cfg.drag_coefficient < 0) {
      throw new Error('aero.drag_coefficient cannot be negative');
    }
  }
}
