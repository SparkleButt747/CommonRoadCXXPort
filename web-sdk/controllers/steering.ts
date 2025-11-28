import { SteeringParameters } from '../models/types.js';

export interface SteeringConfig {
  wheel: {
    max_angle: number;
    max_rate: number;
    nudge_angle: number;
    centering_stiffness: number;
    centering_deadband: number;
  };
  final: {
    min_angle: number;
    max_angle: number;
    max_rate: number;
    actuator_time_constant: number;
    smoothing_time_constant: number;
  };
}

function clamp(value: number, low: number, high: number): number {
  return Math.max(low, Math.min(high, value));
}

function clamp01(value: number): number {
  return clamp(value, 0, 1);
}

export class SteeringWheel {
  private cfg: SteeringConfig['wheel'];
  private limits: SteeringParameters;
  private angle = 0;
  private last: SteeringWheel.Output = { target_angle: 0, angle: 0, rate: 0 };

  constructor(cfg: SteeringConfig['wheel'], limits: SteeringParameters) {
    this.validate(cfg);
    this.cfg = cfg;
    this.limits = limits;
    this.reset(0);
  }

  update(nudge: number, dt: number): SteeringWheel.Output {
    if (!(dt > 0)) {
      throw new Error('dt must be positive');
    }

    const prevAngle = this.angle;
    const leftLimit = this.maxLeft();
    const rightLimit = this.maxRight();

    let clampedNudge = clamp(nudge, -1, 1);
    let angle = this.angle;

    if (Math.abs(clampedNudge) > 1e-9) {
      let target = angle + clampedNudge * this.cfg.nudge_angle;
      target = clamp(target, rightLimit, leftLimit);
      const maxDelta = this.cfg.max_rate * dt;
      const delta = clamp(target - angle, -maxDelta, maxDelta);
      angle += delta;
    } else {
      if (Math.abs(angle) <= this.cfg.centering_deadband) {
        angle = 0;
      } else {
        const effective = Math.sign(angle) * Math.max(0, Math.abs(angle) - this.cfg.centering_deadband);
        let rate = -this.cfg.centering_stiffness * effective;
        rate = clamp(rate, -this.cfg.max_rate, this.cfg.max_rate);
        angle += rate * dt;
        if (angle === 0 || Math.sign(angle) !== Math.sign(this.angle)) {
          angle = 0;
        }
      }
    }

    this.angle = clamp(angle, rightLimit, leftLimit);
    this.last = {
      target_angle: this.angle,
      angle: this.angle,
      rate: (this.angle - prevAngle) / dt,
    };
    return this.last;
  }

  reset(angle = 0): void {
    this.angle = clamp(angle, this.maxRight(), this.maxLeft());
    this.last = { target_angle: this.angle, angle: this.angle, rate: 0 };
  }

  config(): SteeringConfig['wheel'] {
    return this.cfg;
  }

  lastOutput(): SteeringWheel.Output {
    return this.last;
  }

  private maxLeft(): number {
    let limit = this.cfg.max_angle;
    if (this.limits.max > 0) {
      limit = Math.min(limit, this.limits.max);
    }
    return limit;
  }

  private maxRight(): number {
    let limit = -this.cfg.max_angle;
    if (this.limits.min < 0) {
      limit = Math.max(limit, this.limits.min);
    }
    return limit;
  }

  private validate(cfg: SteeringConfig['wheel']): void {
    if (!(cfg.max_angle > 0) || !(cfg.max_rate > 0) || !(cfg.nudge_angle > 0) || !(cfg.centering_stiffness > 0)) {
      throw new Error('Steering wheel configuration is invalid');
    }
    if (cfg.centering_deadband < 0 || cfg.centering_deadband >= cfg.max_angle) {
      throw new Error('Steering centering_deadband must be non-negative and smaller than max_angle');
    }
  }
}

export namespace SteeringWheel {
  export interface Output {
    target_angle: number;
    angle: number;
    rate: number;
  }
}

export class FinalSteerController {
  private cfg: SteeringConfig['final'];
  private limits: SteeringParameters;
  private filteredTarget = 0;
  private last: FinalSteerController.Output = { filtered_target: 0, angle: 0, rate: 0 };

  constructor(cfg: SteeringConfig['final'], limits: SteeringParameters) {
    this.validate(cfg);
    this.cfg = cfg;
    this.limits = limits;
    this.reset(0);
  }

  update(desiredAngle: number, currentAngle: number, dt: number): FinalSteerController.Output {
    if (!(dt > 0)) {
      throw new Error('dt must be positive');
    }
    const minAngle = this.minAngle();
    const maxAngle = this.maxAngle();

    const desiredClamped = clamp(desiredAngle, minAngle, maxAngle);
    const measured = clamp(currentAngle, minAngle, maxAngle);

    if (this.cfg.smoothing_time_constant > 0) {
      const alpha = clamp01(dt / (this.cfg.smoothing_time_constant + dt));
      this.filteredTarget += alpha * (desiredClamped - this.filteredTarget);
    } else {
      this.filteredTarget = desiredClamped;
    }
    this.filteredTarget = clamp(this.filteredTarget, minAngle, maxAngle);

    const tau = this.cfg.actuator_time_constant;
    let rate = (this.filteredTarget - measured) / tau;

    const rateMin = this.combinedRateMin();
    const rateMax = this.combinedRateMax();
    rate = clamp(rate, rateMin, rateMax);

    const maxStep = (maxAngle - measured) / dt;
    const minStep = (minAngle - measured) / dt;
    rate = clamp(rate, minStep, maxStep);

    const newAngle = clamp(measured + rate * dt, minAngle, maxAngle);
    rate = (newAngle - measured) / dt;

    this.last = {
      filtered_target: this.filteredTarget,
      angle: newAngle,
      rate,
    };
    return this.last;
  }

  updateAbsolute(desiredAngle: number, currentAngle: number, dt: number): FinalSteerController.Output {
    return this.update(desiredAngle, currentAngle, dt);
  }

  reset(currentAngle = 0): void {
    const clamped = clamp(currentAngle, this.minAngle(), this.maxAngle());
    this.filteredTarget = clamped;
    this.last = { filtered_target: clamped, angle: clamped, rate: 0 };
  }

  config(): SteeringConfig['final'] {
    return this.cfg;
  }

  lastOutput(): FinalSteerController.Output {
    return this.last;
  }

  minAngle(): number {
    if (this.limits.max <= this.limits.min) return this.cfg.min_angle;
    return Math.max(this.cfg.min_angle, this.limits.min);
  }

  maxAngle(): number {
    if (this.limits.max <= this.limits.min) return this.cfg.max_angle;
    return Math.min(this.cfg.max_angle, this.limits.max);
  }

  private combinedRateMin(): number {
    let rateMin = -this.cfg.max_rate;
    if (this.limits.v_min < 0) {
      rateMin = Math.max(rateMin, this.limits.v_min);
    }
    return rateMin;
  }

  private combinedRateMax(): number {
    let rateMax = this.cfg.max_rate;
    if (this.limits.v_max > 0) {
      rateMax = Math.min(rateMax, this.limits.v_max);
    }
    return rateMax;
  }

  private validate(cfg: SteeringConfig['final']): void {
    if (!(cfg.max_angle > cfg.min_angle) || !(cfg.max_rate > 0) || !(cfg.actuator_time_constant > 0)) {
      throw new Error('Final steering configuration is invalid');
    }
    if (cfg.smoothing_time_constant < 0) {
      throw new Error('smoothing_time_constant cannot be negative');
    }
  }
}

export namespace FinalSteerController {
  export interface Output {
    filtered_target: number;
    angle: number;
    rate: number;
  }
}
