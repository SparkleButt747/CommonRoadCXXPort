#pragma once

#include <string>
#include <vector>

#include "controllers/longitudinal/final_accel_controller.hpp"
#include "controllers/steering_controller.hpp"
#include "simulation/low_speed_safety.hpp"
#include "simulation/model_timing.hpp"
#include "vehicle_parameters.hpp"

namespace velox::telemetry {

struct PoseTelemetry {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

struct VelocityTelemetry {
    double speed       = 0.0;
    double longitudinal = 0.0;
    double lateral      = 0.0;
    double yaw_rate     = 0.0;
    double global_x     = 0.0;
    double global_y     = 0.0;
};

struct AccelerationTelemetry {
    double longitudinal = 0.0;
    double lateral      = 0.0;
};

struct WheelTelemetry {
    double speed               = 0.0;
    double slip_ratio          = 0.0;
    double friction_utilization = 0.0;
};

struct AxleTelemetry {
    double drive_torque  = 0.0;
    double brake_torque  = 0.0;
    double regen_torque  = 0.0;
    double normal_force  = 0.0;
    WheelTelemetry left{};
    WheelTelemetry right{};
};

struct PowertrainTelemetry {
    double total_torque     = 0.0;
    double drive_torque     = 0.0;
    double regen_torque     = 0.0;
    double mechanical_power = 0.0; // +: to wheels, -: from wheels
    double battery_power    = 0.0; // +: discharge, -: regen
    double soc              = 0.0;
};

struct SteeringTelemetry {
    double desired_angle = 0.0;
    double desired_rate  = 0.0;
    double actual_angle  = 0.0;
    double actual_rate   = 0.0;
};

struct ControllerTelemetry {
    double acceleration    = 0.0;
    double throttle        = 0.0;
    double brake           = 0.0;
    double drive_force     = 0.0;
    double brake_force     = 0.0;
    double regen_force     = 0.0;
    double hydraulic_force = 0.0;
    double drag_force      = 0.0;
    double rolling_force   = 0.0;
};

struct DerivedTelemetry {
    double distance_traveled_m   = 0.0;
    double energy_consumed_joules = 0.0;
};

struct SimulationTelemetry {
    PoseTelemetry         pose{};
    VelocityTelemetry     velocity{};
    AccelerationTelemetry acceleration{};
    SteeringTelemetry     steering{};
    ControllerTelemetry   controller{};
    PowertrainTelemetry   powertrain{};
    AxleTelemetry         front_axle{};
    AxleTelemetry         rear_axle{};
    DerivedTelemetry      totals{};
    bool                  low_speed_engaged{false};
};

SimulationTelemetry compute_simulation_telemetry(
    ModelType model,
    const models::VehicleParameters& params,
    const std::vector<double>& state,
    const controllers::longitudinal::ControllerOutput& accel_output,
    const controllers::SteeringWheel::Output& steering_wheel_output,
    const controllers::FinalSteerController::Output& steering_output,
    const simulation::LowSpeedSafety* safety,
    double measured_speed,
    double cumulative_distance_m,
    double cumulative_energy_j);

std::string to_json(const SimulationTelemetry& telemetry);

} // namespace velox::telemetry

