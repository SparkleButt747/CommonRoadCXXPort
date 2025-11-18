#include <cassert>
#include <cmath>
#include <optional>
#include <vector>

#include "controllers/longitudinal/final_accel_controller.hpp"
#include "controllers/steering_controller.hpp"
#include "simulation/low_speed_safety.hpp"
#include "simulation/model_timing.hpp"
#include "telemetry/telemetry.hpp"
#include "vehicle_parameters.hpp"

namespace vt  = velox::telemetry;
namespace vsim = velox::simulation;
namespace vcon = velox::controllers;
namespace vmod = velox::models;

int main()
{
    auto params = vmod::setup_vehicle_parameters(1);

    vsim::LowSpeedSafetyConfig safety_cfg{};
    safety_cfg.normal        = {0.05, 0.25, 1.0, 0.3};
    safety_cfg.drift         = {0.05, 0.25, 1.5, 0.6};
    safety_cfg.stop_speed_epsilon = 0.01;
    safety_cfg.drift_enabled = true;

    vsim::LowSpeedSafety safety(safety_cfg,
                                3,
                                10,
                                5,
                                std::nullopt,
                                {23, 24, 25, 26},
                                2,
                                params.a + params.b,
                                params.b);
    safety.set_drift_enabled(true);

    vcon::longitudinal::ControllerOutput accel_output{};
    accel_output.acceleration = 1.2;
    accel_output.throttle     = 0.35;
    accel_output.drive_force  = 1800.0;

    vcon::SteeringWheel::Output wheel_output{};
    vcon::FinalSteerController::Output steer_output{};

    std::vector<double> calm_state(27, 0.0);
    calm_state[2]  = 0.05; // steering angle
    calm_state[3]  = 8.0;  // longitudinal velocity
    calm_state[4]  = 0.0;  // yaw
    calm_state[5]  = 0.15; // yaw rate
    calm_state[10] = 0.2;  // lateral velocity

    wheel_output.target_angle = calm_state[2];
    steer_output.angle        = calm_state[2];

    auto calm = vt::compute_simulation_telemetry(vsim::ModelType::MB,
                                                 params,
                                                 calm_state,
                                                 accel_output,
                                                 wheel_output,
                                                 steer_output,
                                                 &safety,
                                                 0.0,
                                                 0.0,
                                                 0.0,
                                                 0.0);

    std::vector<double> drift_state = calm_state;
    drift_state[2]  = 0.3;
    drift_state[5]  = 0.65;
    drift_state[10] = 3.0;

    wheel_output.target_angle = drift_state[2];
    steer_output.angle        = drift_state[2];

    auto drift = vt::compute_simulation_telemetry(vsim::ModelType::MB,
                                                  params,
                                                  drift_state,
                                                  accel_output,
                                                  wheel_output,
                                                  steer_output,
                                                  &safety,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0);

    assert(std::abs(drift.traction.slip_angle) > std::abs(calm.traction.slip_angle));
    assert(std::abs(drift.traction.front_slip_angle) > std::abs(calm.traction.front_slip_angle));
    assert(drift.traction.lateral_force_saturation > calm.traction.lateral_force_saturation);
    assert(std::abs(drift.front_axle.left.slip_ratio) > std::abs(calm.front_axle.left.slip_ratio));
    assert(drift.traction.drift_mode);

    return 0;
}
