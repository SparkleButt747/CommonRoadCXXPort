import { AeroConfig, AeroModel } from './aero.js';
import { BrakeBlendOutput, BrakeConfig, BrakeController } from './brake.js';
import { Powertrain, PowertrainConfig, PowertrainOutput } from './powertrain.js';
import { RollingResistance, RollingResistanceConfig } from './rolling.js';

export interface FinalAccelControllerConfig {
  tau_throttle: number;
  tau_brake: number;
  accel_min: number;
  accel_max: number;
  stop_speed_epsilon: number;
}

export interface DriverIntent {
  throttle: number;
  brake: number;
}

export interface ControllerOutput {
  acceleration: number;
  throttle: number;
  brake: number;
  drive_force: number;
  brake_force: number;
  regen_force: number;
  hydraulic_force: number;
  drag_force: number;
  rolling_force: number;
  mechanical_power: number;
  battery_power: number;
  soc: number;
}

export class FinalAccelController {
  private readonly mass: number;
  private readonly wheelRadius: number;
  private readonly powertrain: Powertrain;
  private readonly aero: AeroModel;
  private readonly rolling: RollingResistance;
  private readonly brakes: BrakeController;
  private readonly cfg: FinalAccelControllerConfig;
  private throttleValue = 0;
  private brakeValue = 0;

  constructor(
    vehicleMass: number,
    wheelRadius: number,
    powertrainCfg: PowertrainConfig,
    aeroCfg: AeroConfig,
    rollingCfg: RollingResistanceConfig,
    brakeCfg: BrakeConfig,
    controllerCfg: FinalAccelControllerConfig
  ) {
    if (!Number.isFinite(vehicleMass) || vehicleMass <= 0) {
      throw new Error('vehicle_mass must be positive and finite');
    }
    if (!Number.isFinite(wheelRadius) || wheelRadius <= 0) {
      throw new Error('wheel_radius must be positive and finite');
    }
    this.validate(controllerCfg);
    this.mass = vehicleMass;
    this.wheelRadius = wheelRadius;
    this.powertrain = new Powertrain(powertrainCfg, wheelRadius);
    this.aero = new AeroModel(aeroCfg);
    this.rolling = new RollingResistance(rollingCfg);
    this.brakes = new BrakeController(brakeCfg);
    this.cfg = controllerCfg;
  }

  step(intent: DriverIntent, speed: number, dt: number): ControllerOutput {
    this.applyActuatorDynamics(intent, dt);
    const brakeRequest = this.brakeValue;
    const throttleCommand = this.throttleValue * (1 - Math.min(brakeRequest, 1));

    const availableRegenForce = this.powertrain.available_regen_torque(speed) / this.wheelRadius;
    const brakeOutput: BrakeBlendOutput = this.brakes.blend(brakeRequest, speed, availableRegenForce);

    const regenTorqueRequest = brakeOutput.regen_force * this.wheelRadius;
    const powertrainOutput: PowertrainOutput = this.powertrain.step(throttleCommand, regenTorqueRequest, speed, dt);

    let driveForce = powertrainOutput.drive_torque / this.wheelRadius;
    let regenForce = powertrainOutput.regen_torque / this.wheelRadius;
    if (regenForce > brakeOutput.regen_force + 1e-6) {
      regenForce = brakeOutput.regen_force;
    }
    const hydraulicForce = Math.max(0, brakeOutput.hydraulic_force + (brakeOutput.regen_force - regenForce));
    const brakeForce = hydraulicForce + regenForce;

    const dragForce = this.aero.drag_force(speed);
    const downforce = this.aero.downforce(speed);
    const normalForce = this.mass * 9.81 + downforce;
    const rollingForce = this.rolling.force(speed, normalForce);

    const netForce = driveForce - brakeForce + dragForce + rollingForce;
    let acceleration = netForce / this.mass;
    acceleration = clamp(acceleration, this.cfg.accel_min, this.cfg.accel_max);
    if (this.cfg.stop_speed_epsilon > 0 && brakeRequest > 1e-6 && Math.abs(speed) <= this.cfg.stop_speed_epsilon && acceleration < 0) {
      acceleration = 0;
    }

    return {
      acceleration,
      throttle: throttleCommand,
      brake: brakeRequest,
      drive_force: driveForce,
      brake_force: brakeForce,
      regen_force: regenForce,
      hydraulic_force: hydraulicForce,
      drag_force: dragForce,
      rolling_force: rollingForce,
      mechanical_power: powertrainOutput.mechanical_power,
      battery_power: powertrainOutput.battery_power,
      soc: this.powertrain.soc(),
    };
  }

  reset(): void {
    this.throttleValue = 0;
    this.brakeValue = 0;
    this.powertrain.reset();
  }

  private applyActuatorDynamics(intent: DriverIntent, dt: number): void {
    const tauThrottle = Math.max(this.cfg.tau_throttle, 1e-3);
    const tauBrake = Math.max(this.cfg.tau_brake, 1e-3);
    const throttleTarget = clamp(intent.throttle, 0, 1);
    const brakeTarget = clamp(intent.brake, 0, 1);

    if (brakeTarget > 0) {
      this.throttleValue = 0;
    } else {
      this.throttleValue += dt / tauThrottle * (throttleTarget - this.throttleValue);
      this.throttleValue = clamp(this.throttleValue, 0, 1);
    }
    this.brakeValue += dt / tauBrake * (brakeTarget - this.brakeValue);
    this.brakeValue = clamp(this.brakeValue, 0, 1);
  }

  private validate(cfg: FinalAccelControllerConfig): void {
    if (!Number.isFinite(cfg.tau_throttle) || cfg.tau_throttle <= 0) {
      throw new Error('final_accel_controller.tau_throttle must be positive');
    }
    if (!Number.isFinite(cfg.tau_brake) || cfg.tau_brake <= 0) {
      throw new Error('final_accel_controller.tau_brake must be positive');
    }
    if (!Number.isFinite(cfg.accel_min) || !Number.isFinite(cfg.accel_max) || cfg.accel_max < cfg.accel_min) {
      throw new Error('final_accel_controller accel bounds invalid');
    }
    if (!Number.isFinite(cfg.stop_speed_epsilon) || cfg.stop_speed_epsilon < 0) {
      throw new Error('final_accel_controller.stop_speed_epsilon must be non-negative');
    }
  }
}

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}
