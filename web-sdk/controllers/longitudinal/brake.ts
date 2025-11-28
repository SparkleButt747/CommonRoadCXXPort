export interface BrakeConfig {
  max_force: number;
  max_regen_force: number;
  min_regen_speed: number;
}

export interface BrakeBlendOutput {
  regen_force: number;
  hydraulic_force: number;
  total_force: number;
}

export class BrakeController {
  private readonly cfg: BrakeConfig;

  constructor(config: BrakeConfig) {
    this.validate(config);
    this.cfg = config;
  }

  blend(brakePedal: number, speed: number, availableRegenForce: number): BrakeBlendOutput {
    let pedal = Math.min(Math.max(brakePedal, 0), 1);
    let regenAvailable = Math.max(0, availableRegenForce);

    const totalForce = Math.min(this.cfg.max_force, pedal * this.cfg.max_force);
    let regenCapacity = Math.min(totalForce, Math.min(pedal * this.cfg.max_regen_force, regenAvailable));
    regenCapacity = Math.max(0, regenCapacity);

    const minRegenSpeed = Math.max(0, this.cfg.min_regen_speed);
    let weight = 1;
    if (minRegenSpeed > 0) {
      const speedRatio = Math.abs(speed) / minRegenSpeed;
      weight = Math.min(Math.max(speedRatio, 0), 1);
    }

    const regen_force = regenCapacity * weight;
    const hydraulic_force = Math.max(0, totalForce - regen_force);
    return { regen_force, hydraulic_force, total_force: totalForce };
  }

  private validate(cfg: BrakeConfig): void {
    const check = (value: number, name: string) => {
      if (!Number.isFinite(value) || value < 0) {
        throw new Error(`${name} must be non-negative and finite`);
      }
    };
    check(cfg.max_force, 'brake.max_force');
    check(cfg.max_regen_force, 'brake.max_regen_force');
    check(cfg.min_regen_speed, 'brake.min_regen_speed');
  }
}
