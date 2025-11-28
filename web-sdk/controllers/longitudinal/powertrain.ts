export interface PowertrainConfig {
  max_drive_torque: number;
  max_regen_torque: number;
  max_power: number;
  drive_efficiency: number;
  regen_efficiency: number;
  min_soc: number;
  max_soc: number;
  initial_soc: number;
  battery_capacity_kwh: number;
}

export interface PowertrainOutput {
  total_torque: number;
  drive_torque: number;
  regen_torque: number;
  mechanical_power: number;
  battery_power: number;
}

export class Powertrain {
  private readonly cfg: PowertrainConfig;
  private readonly wheelRadius: number;
  private readonly capacityJoules: number;
  private socValue: number;

  constructor(config: PowertrainConfig, wheelRadius: number) {
    this.validate(config, wheelRadius);
    this.cfg = config;
    this.wheelRadius = wheelRadius;
    this.capacityJoules = config.battery_capacity_kwh * 3.6e6;
    this.socValue = config.initial_soc;
  }

  available_drive_torque(speed: number): number {
    if (this.socValue <= this.cfg.min_soc) return 0;
    const limit = this.torque_power_limited(speed);
    return Math.min(Math.max(limit, 0), this.cfg.max_drive_torque);
  }

  available_regen_torque(speed: number): number {
    if (this.socValue >= this.cfg.max_soc) return 0;
    if (Math.abs(speed) < 1e-3) return 0;
    const limit = this.torque_power_limited(speed);
    return Math.min(Math.max(limit, 0), this.cfg.max_regen_torque);
  }

  step(throttle: number, regenTorqueRequest: number, speed: number, dt: number): PowertrainOutput {
    const clampedThrottle = Math.min(Math.max(throttle, 0), 1);
    const regenRequest = Math.max(0, regenTorqueRequest);

    const driveLimit = this.available_drive_torque(speed);
    const driveTorque = Math.min(clampedThrottle * this.cfg.max_drive_torque, driveLimit);
    const regenLimit = this.available_regen_torque(speed);
    const regenTorque = Math.min(regenRequest, regenLimit);

    const wheelSpeed = speed / this.wheelRadius;
    let mechanicalDrivePower = driveTorque * wheelSpeed;
    let mechanicalRegenPower = -regenTorque * wheelSpeed;

    let batteryPower = 0;
    if (mechanicalDrivePower > 0) {
      batteryPower += mechanicalDrivePower / Math.max(this.cfg.drive_efficiency, 1e-6);
    } else {
      mechanicalDrivePower = 0;
    }
    if (mechanicalRegenPower < 0) {
      batteryPower += mechanicalRegenPower * this.cfg.regen_efficiency;
    } else {
      mechanicalRegenPower = 0;
    }

    const socDelta = dt > 0 ? -batteryPower * dt / this.capacityJoules : 0;
    this.socValue = Math.min(Math.max(this.socValue + socDelta, this.cfg.min_soc), this.cfg.max_soc);

    const totalTorque = driveTorque - regenTorque;
    const mechanical_power = mechanicalDrivePower + mechanicalRegenPower;
    return { total_torque: totalTorque, drive_torque: driveTorque, regen_torque: regenTorque, mechanical_power, battery_power: batteryPower };
  }

  reset(): void {
    this.socValue = this.cfg.initial_soc;
  }

  soc(): number {
    return this.socValue;
  }

  private torque_power_limited(speed: number): number {
    if (this.cfg.max_power <= 0) return this.cfg.max_drive_torque;
    const wheelSpeed = Math.abs(speed) / this.wheelRadius;
    if (wheelSpeed < 1e-6) return this.cfg.max_drive_torque;
    return this.cfg.max_power / wheelSpeed;
  }

  private validate(config: PowertrainConfig, wheelRadius: number): void {
    const finiteNonNeg = (value: number, name: string) => {
      if (!Number.isFinite(value) || value < 0) {
        throw new Error(`${name} must be non-negative and finite`);
      }
    };
    finiteNonNeg(config.max_drive_torque, 'powertrain.max_drive_torque');
    finiteNonNeg(config.max_regen_torque, 'powertrain.max_regen_torque');
    finiteNonNeg(config.max_power, 'powertrain.max_power');
    finiteNonNeg(config.drive_efficiency, 'powertrain.drive_efficiency');
    finiteNonNeg(config.regen_efficiency, 'powertrain.regen_efficiency');
    if (config.drive_efficiency <= 0 || config.drive_efficiency > 1) {
      throw new Error('powertrain.drive_efficiency must be in (0,1]');
    }
    if (config.regen_efficiency < 0 || config.regen_efficiency > 1) {
      throw new Error('powertrain.regen_efficiency must be in [0,1]');
    }
    if (!Number.isFinite(config.min_soc) || !Number.isFinite(config.max_soc) || !Number.isFinite(config.initial_soc)) {
      throw new Error('powertrain SOC bounds must be finite');
    }
    if (config.min_soc < 0 || config.max_soc > 1 || !(config.min_soc <= config.initial_soc && config.initial_soc <= config.max_soc)) {
      throw new Error('SOC bounds must satisfy 0 <= min <= initial <= max <= 1');
    }
    if (!Number.isFinite(config.battery_capacity_kwh) || config.battery_capacity_kwh <= 0) {
      throw new Error('battery_capacity_kwh must be positive');
    }
    if (!Number.isFinite(wheelRadius) || wheelRadius <= 0) {
      throw new Error('wheel_radius must be positive and finite');
    }
  }
}
