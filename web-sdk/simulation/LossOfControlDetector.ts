export interface MetricThreshold {
  threshold: number;
  rate: number;
}

export interface LossOfControlConfig {
  yaw_rate: MetricThreshold;
  slip_angle: MetricThreshold;
  lateral_accel: MetricThreshold;
  slip_ratio: MetricThreshold;
}

interface MetricState {
  value: number;
  hasPrevious: boolean;
}

export class LossOfControlDetector {
  private yawRate: MetricState = { value: 0, hasPrevious: false };
  private slipAngle: MetricState = { value: 0, hasPrevious: false };
  private lateralAccel: MetricState = { value: 0, hasPrevious: false };
  private wheels: MetricState[] = [];
  private severity = 0;

  constructor(private readonly config: LossOfControlConfig) {}

  reset(): void {
    this.yawRate = { value: 0, hasPrevious: false };
    this.slipAngle = { value: 0, hasPrevious: false };
    this.lateralAccel = { value: 0, hasPrevious: false };
    this.wheels = [];
    this.severity = 0;
  }

  update(dt: number, yawRate: number, slipAngle: number, lateralAccel: number, wheelSlipRatios: number[]): number {
    if (!(dt > 0)) {
      throw new Error('LossOfControlDetector requires positive dt');
    }
    if (this.wheels.length !== wheelSlipRatios.length) {
      this.wheels = wheelSlipRatios.map(() => ({ value: 0, hasPrevious: false }));
    }

    const scores = [
      this.evaluateMetric(dt, yawRate, this.yawRate, this.config.yaw_rate),
      this.evaluateMetric(dt, slipAngle, this.slipAngle, this.config.slip_angle),
      this.evaluateMetric(dt, lateralAccel, this.lateralAccel, this.config.lateral_accel),
    ];

    wheelSlipRatios.forEach((ratio, idx) => {
      scores.push(this.evaluateMetric(dt, ratio, this.wheels[idx], this.config.slip_ratio));
    });

    this.severity = Math.max(0, ...scores);
    return this.severity;
  }

  getSeverity(): number {
    return this.severity;
  }

  private evaluateMetric(dt: number, value: number, state: MetricState, limits: MetricThreshold): number {
    let severity = 0;
    if (state.hasPrevious) {
      const delta = Math.abs(value - state.value);
      const rate = delta / Math.max(dt, 1e-9);
      const mag = Math.abs(value);
      if (mag >= limits.threshold && rate >= limits.rate) {
        const magScore = (mag - limits.threshold) / limits.threshold;
        const rateScore = (rate - limits.rate) / limits.rate;
        severity = Math.max(0, 0.5 * magScore + 0.5 * rateScore);
      }
    }
    state.value = value;
    state.hasPrevious = true;
    return severity;
  }
}
