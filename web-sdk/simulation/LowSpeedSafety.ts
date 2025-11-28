import { SafetyStage } from '../telemetry/index.js';

export interface LowSpeedSafetyProfile {
  engage_speed: number;
  release_speed: number;
  yaw_rate_limit: number;
  slip_angle_limit: number;
}

export interface LowSpeedSafetyConfig {
  normal: LowSpeedSafetyProfile;
  drift: LowSpeedSafetyProfile;
  stop_speed_epsilon: number;
  drift_enabled: boolean;
}

interface MonitoredMetrics {
  speed: number;
  severity: number;
  transition_blend: number;
  drift_mode: boolean;
  near_stop: boolean;
  yaw_state?: number;
  slip_state?: number;
  yaw_target?: number;
  slip_target?: number;
  lateral_target?: number;
  velocity_heading?: number;
}

interface SafetyDecision {
  mode: SafetyStage;
  latch_active: boolean;
  transition_blend: number;
  severity_trip: boolean;
}

export interface LowSpeedIndices {
  longitudinalIndex?: number;
  lateralIndex?: number;
  yawRateIndex?: number;
  slipIndex?: number;
  wheelSpeedIndices?: number[];
  steeringIndex?: number;
  wheelbase?: number;
  rearLength?: number;
}

export class LowSpeedSafety {
  private engaged = false;
  private driftEnabled: boolean;

  constructor(
    private readonly config: LowSpeedSafetyConfig,
    private readonly indices: LowSpeedIndices
  ) {
    this.driftEnabled = config.drift_enabled;
    this.validateConfig(config);
  }

  reset(): void {
    this.engaged = false;
  }

  setDriftEnabled(enabled: boolean): void {
    this.driftEnabled = enabled;
  }

  status(state: number[], speed: number) {
    const profile = this.activeProfile();
    const metrics = this.monitor(state, speed, profile);
    const decision = this.decide(metrics, profile, false);
    return {
      severity: metrics.severity,
      transition_blend: metrics.transition_blend,
      drift_mode: metrics.drift_mode,
      detector_forced: metrics.severity > 1,
      latch_active: this.engaged || metrics.severity > 1,
      stage: decision.mode,
    };
  }

  apply(state: number[], speed: number, updateLatch = true): void {
    const profile = this.activeProfile();
    const metrics = this.monitor(state, speed, profile);
    const decision = this.decide(metrics, profile, updateLatch);
    this.clampState(state, metrics, decision, profile);
  }

  longitudinalIndex(): number | undefined {
    return this.indices.longitudinalIndex;
  }

  private activeProfile(): LowSpeedSafetyProfile {
    return this.driftEnabled ? this.config.drift : this.config.normal;
  }

  private validateConfig(cfg: LowSpeedSafetyConfig): void {
    const validateProfile = (profile: LowSpeedSafetyProfile, name: string) => {
      if (profile.engage_speed < 0 || profile.release_speed <= 0 || profile.release_speed < profile.engage_speed) {
        throw new Error(`LowSpeedSafety profile ${name} has invalid engage/release speeds`);
      }
      if (!(profile.yaw_rate_limit > 0) || !(profile.slip_angle_limit > 0)) {
        throw new Error(`LowSpeedSafety profile ${name} requires positive limits`);
      }
    };
    validateProfile(cfg.normal, 'normal');
    validateProfile(cfg.drift, 'drift');
    if (cfg.stop_speed_epsilon < 0) {
      throw new Error('stop_speed_epsilon must be non-negative');
    }
  }

  private monitor(state: number[], speed: number, profile: LowSpeedSafetyProfile): MonitoredMetrics {
    const metrics: MonitoredMetrics = {
      speed,
      severity: 0,
      transition_blend: this.preLatchBlend(speed, profile),
      drift_mode: this.driftEnabled,
      near_stop: Math.abs(speed) <= this.config.stop_speed_epsilon,
    };

    if (this.indexInBounds(this.indices.yawRateIndex, state)) {
      metrics.yaw_state = state[this.indices.yawRateIndex as number];
    }
    if (this.indexInBounds(this.indices.slipIndex, state)) {
      metrics.slip_state = state[this.indices.slipIndex as number];
    }

    metrics.yaw_target = this.kinematicYawRate(state, speed);
    metrics.slip_target = this.kinematicSlip(state, speed);
    metrics.lateral_target = this.kinematicLateralVelocity(state, speed);
    metrics.velocity_heading = this.velocitySlip(state);

    const yawLimit = Math.max(profile.yaw_rate_limit, 1e-6);
    const slipLimit = Math.max(profile.slip_angle_limit, 1e-6);

    const yawRatio = metrics.yaw_state !== undefined ? Math.abs(metrics.yaw_state) / yawLimit : 0;
    const slipRatio = metrics.slip_state !== undefined ? Math.abs(metrics.slip_state) / slipLimit : 0;
    metrics.severity = Math.max(yawRatio, slipRatio);
    return metrics;
  }

  private decide(metrics: MonitoredMetrics, profile: LowSpeedSafetyProfile, updateLatch: boolean): SafetyDecision {
    const severityTrip = metrics.severity > 1;

    if (updateLatch) {
      if (this.engaged) {
        if (metrics.speed > profile.release_speed && !severityTrip) {
          this.engaged = false;
        }
      } else if (metrics.speed < profile.engage_speed || severityTrip) {
        this.engaged = true;
      }
    }

    const decision: SafetyDecision = {
      severity_trip: severityTrip,
      transition_blend: metrics.transition_blend,
      latch_active: this.engaged || severityTrip,
      mode: SafetyStage.Normal,
    };

    if (decision.latch_active) {
      decision.mode = SafetyStage.Emergency;
    } else if (metrics.transition_blend > 0) {
      decision.mode = SafetyStage.Transition;
    }

    return decision;
  }

  private clampState(
    state: number[],
    metrics: MonitoredMetrics,
    decision: SafetyDecision,
    profile: LowSpeedSafetyProfile
  ): void {
    const driftMode = metrics.drift_mode;
    const allowUnclamped = driftMode && decision.mode === SafetyStage.Normal && decision.transition_blend <= 0;
    const wheelLatch = decision.latch_active || metrics.speed < profile.engage_speed || decision.transition_blend > 0;

    let yawTarget = metrics.yaw_target;
    let slipTarget = metrics.slip_target;
    let lateralTarget = metrics.lateral_target;
    const velocityHeading = metrics.velocity_heading;

    if (decision.mode === SafetyStage.Emergency) {
      const betaRef = velocityHeading ?? 0;
      const slipCommand = velocityHeading !== undefined && !metrics.near_stop ? betaRef : 0;
      yawTarget = 0;
      slipTarget = slipCommand;
      if (velocityHeading !== undefined && !metrics.near_stop) {
        lateralTarget = metrics.speed * Math.sin(slipCommand);
      } else {
        lateralTarget = 0;
      }
    }

    if (this.indexInBounds(this.indices.yawRateIndex, state)) {
      const idx = this.indices.yawRateIndex as number;
      if (decision.mode === SafetyStage.Emergency) {
        const limit = profile.yaw_rate_limit;
        const target = yawTarget ?? 0;
        state[idx] = this.clamp(target, -limit, limit);
      } else if (!allowUnclamped) {
        const limit = this.scaledLimit(profile.yaw_rate_limit, metrics.speed, profile);
        let value = this.clamp(state[idx], -limit, limit);
        if (decision.transition_blend > 0 && yawTarget !== undefined) {
          const target = this.clamp(yawTarget, -limit, limit);
          value = (1 - decision.transition_blend) * value + decision.transition_blend * target;
        }
        state[idx] = value;
      }
    }

    if (this.indexInBounds(this.indices.lateralIndex, state)) {
      const idx = this.indices.lateralIndex as number;
      const value = state[idx];
      if (decision.mode === SafetyStage.Emergency) {
        if (lateralTarget !== undefined) {
          state[idx] = lateralTarget;
        } else {
          const limit = this.config.stop_speed_epsilon;
          state[idx] = this.clamp(value, -limit, limit);
        }
      } else if (Math.abs(value) <= this.config.stop_speed_epsilon) {
        state[idx] = 0;
      }
    }

    if (this.indexInBounds(this.indices.slipIndex, state)) {
      const idx = this.indices.slipIndex as number;
      if (decision.mode === SafetyStage.Emergency) {
        const limit = profile.slip_angle_limit;
        const target = slipTarget ?? 0;
        state[idx] = this.clamp(target, -limit, limit);
      } else if (!allowUnclamped) {
        const limit = this.scaledLimit(profile.slip_angle_limit, metrics.speed, profile);
        let value = this.clamp(state[idx], -limit, limit);
        let target = 0;
        if (velocityHeading !== undefined && !metrics.near_stop) {
          target = this.clamp(velocityHeading, -limit, limit);
        }
        if (decision.transition_blend > 0) {
          value = (1 - decision.transition_blend) * value + decision.transition_blend * target;
        }
        state[idx] = value;
      }
    }

    if (this.indices.wheelSpeedIndices && this.indices.wheelSpeedIndices.length > 0) {
      for (const rawIdx of this.indices.wheelSpeedIndices) {
        if (!this.indexInBounds(rawIdx, state)) continue;
        const idx = rawIdx;
        const value = state[idx];
        if (value <= 0) {
          state[idx] = 0;
        } else if (wheelLatch && value <= this.config.stop_speed_epsilon) {
          state[idx] = 0;
        }
      }
    }
  }

  private indexInBounds(index: number | undefined, state: number[]): boolean {
    return index !== undefined && index >= 0 && index < state.length;
  }

  private clamp(value: number, min: number, max: number): number {
    return Math.max(min, Math.min(max, value));
  }

  private kinematicBeta(state: number[]): number | undefined {
    const { steeringIndex, wheelbase, rearLength } = this.indices;
    if (steeringIndex === undefined || wheelbase === undefined || rearLength === undefined) return undefined;
    if (!this.indexInBounds(steeringIndex, state)) return undefined;
    if (wheelbase <= 0) return undefined;
    const delta = state[steeringIndex];
    const ratio = rearLength / wheelbase;
    return Math.atan(Math.tan(delta) * ratio);
  }

  private kinematicYawRate(state: number[], speed: number): number | undefined {
    const beta = this.kinematicBeta(state);
    const { steeringIndex, wheelbase } = this.indices;
    if (beta === undefined || steeringIndex === undefined || wheelbase === undefined) return undefined;
    if (!this.indexInBounds(steeringIndex, state)) return undefined;
    if (Math.abs(speed) <= 1e-9) return 0;
    const delta = state[steeringIndex];
    if (!(wheelbase > 0)) return undefined;
    return speed * Math.cos(beta) * Math.tan(delta) / wheelbase;
  }

  private kinematicSlip(state: number[], speed: number): number | undefined {
    void speed;
    return this.kinematicBeta(state);
  }

  private kinematicLateralVelocity(state: number[], speed: number): number | undefined {
    const beta = this.kinematicBeta(state);
    if (beta === undefined) return undefined;
    return speed * Math.sin(beta);
  }

  private velocitySlip(state: number[]): number | undefined {
    const { longitudinalIndex, lateralIndex, slipIndex } = this.indices;
    if (longitudinalIndex === undefined || lateralIndex === undefined) {
      if (this.indexInBounds(slipIndex, state)) {
        return state[slipIndex as number];
      }
      return undefined;
    }
    if (!this.indexInBounds(longitudinalIndex, state) || !this.indexInBounds(lateralIndex, state)) {
      return undefined;
    }
    const longitudinal = state[longitudinalIndex];
    const lateral = state[lateralIndex];
    if (Math.abs(longitudinal) <= 1e-9 && Math.abs(lateral) <= 1e-9) {
      return 0;
    }
    return Math.atan2(lateral, longitudinal);
  }

  private preLatchBlend(speed: number, profile: LowSpeedSafetyProfile): number {
    const band = Math.max(profile.release_speed - profile.engage_speed, 0);
    const upper = profile.release_speed + band;
    const lower = profile.release_speed;
    if (upper <= lower || speed >= upper) {
      return 0;
    }
    if (speed <= lower) {
      return 1;
    }
    const ratio = (upper - speed) / (upper - lower);
    return this.clamp(ratio, 0, 1);
  }

  private scaledLimit(limit: number, speed: number, profile: LowSpeedSafetyProfile): number {
    if (speed >= profile.release_speed || profile.release_speed <= 0) {
      return limit;
    }
    const ratio = this.clamp(speed / Math.max(profile.release_speed, 1e-6), 0, 1);
    const minLimit = Math.max(this.config.stop_speed_epsilon, 1e-6);
    return this.clamp(limit * ratio, minLimit, limit);
  }
}
