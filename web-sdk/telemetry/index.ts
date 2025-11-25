/**
 * Telemetry model interfaces mirroring the C++ schema.
 * Units are noted per field for interoperability with the native daemon.
 */

/** Pose of the vehicle. */
export interface PoseTelemetry {
  /** Position along the global X axis [m]. */
  x: number;
  /** Position along the global Y axis [m]. */
  y: number;
  /** Heading (yaw) in the global frame [rad]. */
  yaw: number;
}

export class PoseTelemetryState implements PoseTelemetry {
  x = 0.0;
  y = 0.0;
  yaw = 0.0;
}

/** Body and world-frame velocities. */
export interface VelocityTelemetry {
  /** Scalar speed [m/s]. */
  speed: number;
  /** Longitudinal velocity in the body frame [m/s]. */
  longitudinal: number;
  /** Lateral velocity in the body frame [m/s]. */
  lateral: number;
  /** Yaw rate [rad/s]. */
  yaw_rate: number;
  /** Global X velocity [m/s]. */
  global_x: number;
  /** Global Y velocity [m/s]. */
  global_y: number;
}

export class VelocityTelemetryState implements VelocityTelemetry {
  speed = 0.0;
  longitudinal = 0.0;
  lateral = 0.0;
  yaw_rate = 0.0;
  global_x = 0.0;
  global_y = 0.0;
}

/** Linear accelerations. */
export interface AccelerationTelemetry {
  /** Longitudinal acceleration [m/s^2]. */
  longitudinal: number;
  /** Lateral acceleration [m/s^2]. */
  lateral: number;
}

export class AccelerationTelemetryState implements AccelerationTelemetry {
  longitudinal = 0.0;
  lateral = 0.0;
}

/** Tire and body slip information. */
export interface TractionTelemetry {
  /** Body slip angle beta [rad]. */
  slip_angle: number;
  /** Front axle slip angle [rad]. */
  front_slip_angle: number;
  /** Rear axle slip angle [rad]. */
  rear_slip_angle: number;
  /** Ratio of lateral force to available friction [-]. */
  lateral_force_saturation: number;
  /** True when drift mode is active. */
  drift_mode: boolean;
}

export class TractionTelemetryState implements TractionTelemetry {
  slip_angle = 0.0;
  front_slip_angle = 0.0;
  rear_slip_angle = 0.0;
  lateral_force_saturation = 0.0;
  drift_mode = false;
}

/** Individual wheel telemetry. */
export interface WheelTelemetry {
  /** Wheel speed along the rolling direction [m/s]. */
  speed: number;
  /** Slip ratio relative to reference wheel speed [-]. */
  slip_ratio: number;
  /** Utilization of available friction [-]. */
  friction_utilization: number;
}

export class WheelTelemetryState implements WheelTelemetry {
  speed = 0.0;
  slip_ratio = 0.0;
  friction_utilization = 0.0;
}

/** Per-axle forces and torques. */
export interface AxleTelemetry {
  /** Drive torque applied to the axle [Nm]. */
  drive_torque: number;
  /** Hydraulic brake torque [Nm]. */
  brake_torque: number;
  /** Regenerative brake torque [Nm]. */
  regen_torque: number;
  /** Normal force on the axle [N]. */
  normal_force: number;
  /** Left wheel telemetry. */
  left: WheelTelemetry;
  /** Right wheel telemetry. */
  right: WheelTelemetry;
}

export class AxleTelemetryState implements AxleTelemetry {
  drive_torque = 0.0;
  brake_torque = 0.0;
  regen_torque = 0.0;
  normal_force = 0.0;
  left: WheelTelemetry = new WheelTelemetryState();
  right: WheelTelemetry = new WheelTelemetryState();
}

/** Powertrain outputs. */
export interface PowertrainTelemetry {
  /** Net torque (drive - regen) [Nm]. */
  total_torque: number;
  /** Applied drive torque [Nm]. */
  drive_torque: number;
  /** Applied regenerative torque [Nm]. */
  regen_torque: number;
  /** Mechanical power transferred to/from the wheels [W]. */
  mechanical_power: number;
  /** Battery power (+ discharge, - regen) [W]. */
  battery_power: number;
  /** State-of-charge fraction [0-1]. */
  soc: number;
}

export class PowertrainTelemetryState implements PowertrainTelemetry {
  total_torque = 0.0;
  drive_torque = 0.0;
  regen_torque = 0.0;
  mechanical_power = 0.0;
  battery_power = 0.0;
  soc = 0.0;
}

/** Steering actuator telemetry. */
export interface SteeringTelemetry {
  /** Steering wheel commanded angle [rad]. */
  desired_angle: number;
  /** Steering wheel rate command [rad/s]. */
  desired_rate: number;
  /** Actual wheel angle [rad]. */
  actual_angle: number;
  /** Actual wheel rate [rad/s]. */
  actual_rate: number;
}

export class SteeringTelemetryState implements SteeringTelemetry {
  desired_angle = 0.0;
  desired_rate = 0.0;
  actual_angle = 0.0;
  actual_rate = 0.0;
}

/** Acceleration controller telemetry. */
export interface ControllerTelemetry {
  /** Acceleration output [m/s^2]. */
  acceleration: number;
  /** Actuated throttle command [-]. */
  throttle: number;
  /** Actuated brake command [-]. */
  brake: number;
  /** Drive force [N]. */
  drive_force: number;
  /** Total brake force [N]. */
  brake_force: number;
  /** Regenerative braking force [N]. */
  regen_force: number;
  /** Hydraulic brake force [N]. */
  hydraulic_force: number;
  /** Aerodynamic drag force [N]. */
  drag_force: number;
  /** Rolling resistance force [N]. */
  rolling_force: number;
}

export class ControllerTelemetryState implements ControllerTelemetry {
  acceleration = 0.0;
  throttle = 0.0;
  brake = 0.0;
  drive_force = 0.0;
  brake_force = 0.0;
  regen_force = 0.0;
  hydraulic_force = 0.0;
  drag_force = 0.0;
  rolling_force = 0.0;
}

/** Aggregate simulation totals. */
export interface DerivedTelemetry {
  /** Cumulative distance traveled [m]. */
  distance_traveled_m: number;
  /** Cumulative energy consumed [J]. */
  energy_consumed_joules: number;
  /** Elapsed simulation time [s]. */
  simulation_time_s: number;
}

export class DerivedTelemetryState implements DerivedTelemetry {
  distance_traveled_m = 0.0;
  energy_consumed_joules = 0.0;
  simulation_time_s = 0.0;
}

/**
 * Safety stage reported by the low-speed safety system.
 */
export enum SafetyStage {
  Normal = 'normal',
  Transition = 'transition',
  Emergency = 'emergency',
}

/** Complete telemetry bundle matching the native layout. */
export interface SimulationTelemetry {
  pose: PoseTelemetry;
  velocity: VelocityTelemetry;
  acceleration: AccelerationTelemetry;
  traction: TractionTelemetry;
  steering: SteeringTelemetry;
  controller: ControllerTelemetry;
  powertrain: PowertrainTelemetry;
  front_axle: AxleTelemetry;
  rear_axle: AxleTelemetry;
  totals: DerivedTelemetry;
  detector_severity: number;
  safety_stage: SafetyStage;
  detector_forced: boolean;
  low_speed_engaged: boolean;
}

export class SimulationTelemetryState implements SimulationTelemetry {
  pose: PoseTelemetry = new PoseTelemetryState();
  velocity: VelocityTelemetry = new VelocityTelemetryState();
  acceleration: AccelerationTelemetry = new AccelerationTelemetryState();
  traction: TractionTelemetry = new TractionTelemetryState();
  steering: SteeringTelemetry = new SteeringTelemetryState();
  controller: ControllerTelemetry = new ControllerTelemetryState();
  powertrain: PowertrainTelemetry = new PowertrainTelemetryState();
  front_axle: AxleTelemetry = new AxleTelemetryState();
  rear_axle: AxleTelemetry = new AxleTelemetryState();
  totals: DerivedTelemetry = new DerivedTelemetryState();
  detector_severity = 0.0;
  safety_stage: SafetyStage = SafetyStage.Normal;
  detector_forced = false;
  low_speed_engaged = false;
}

/**
 * Convert SimulationTelemetry to a JSON-serializable object mirroring the C++ `to_json` layout.
 */
export function toJson(telemetry: SimulationTelemetry): Record<string, unknown> {
  return {
    pose: { ...telemetry.pose },
    velocity: { ...telemetry.velocity },
    acceleration: { ...telemetry.acceleration },
    traction: { ...telemetry.traction },
    steering: { ...telemetry.steering },
    controller: { ...telemetry.controller },
    powertrain: { ...telemetry.powertrain },
    front_axle: {
      ...telemetry.front_axle,
      left: { ...telemetry.front_axle.left },
      right: { ...telemetry.front_axle.right },
    },
    rear_axle: {
      ...telemetry.rear_axle,
      left: { ...telemetry.rear_axle.left },
      right: { ...telemetry.rear_axle.right },
    },
    totals: { ...telemetry.totals },
    low_speed_engaged: telemetry.low_speed_engaged,
    detector_severity: telemetry.detector_severity,
    safety_stage: telemetry.safety_stage,
    detector_forced: telemetry.detector_forced,
  };
}
