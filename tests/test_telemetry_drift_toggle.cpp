#include <cassert>
#include <optional>
#include <vector>

#include "controllers/longitudinal/final_accel_controller.hpp"
#include "controllers/steering_controller.hpp"
#include "simulation/low_speed_safety.hpp"
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
    safety_cfg.normal.engage_speed     = 0.25;
    safety_cfg.normal.release_speed    = 1.0;
    safety_cfg.normal.yaw_rate_limit   = 0.8;
    safety_cfg.normal.slip_angle_limit = 0.6;
    safety_cfg.drift.engage_speed      = 0.15;
    safety_cfg.drift.release_speed     = 1.2;
    safety_cfg.drift.yaw_rate_limit    = 1.4;
    safety_cfg.drift.slip_angle_limit  = 1.0;
    safety_cfg.stop_speed_epsilon      = 0.01;
    safety_cfg.drift_enabled           = false;

    vsim::LowSpeedSafety safety(safety_cfg,
                                3,
                                10,
                                5,
                                std::nullopt,
                                {23, 24, 25, 26},
                                2,
                                params.a + params.b,
                                params.b);

    vcon::longitudinal::ControllerOutput accel_output{};
    vcon::SteeringWheel::Output          wheel_output{};
    vcon::FinalSteerController::Output   steer_output{};

    std::vector<double> state(27, 0.0);
    state[2]  = 0.05; // steering angle
    state[3]  = 0.1;  // longitudinal velocity
    state[4]  = 0.0;  // yaw
    state[5]  = 0.05; // yaw rate
    state[10] = 0.05; // lateral velocity

    safety.apply(state, 0.1, true);

    auto telem = vt::compute_simulation_telemetry(vsim::ModelType::MB,
                                                  params,
                                                  state,
                                                  accel_output,
                                                  wheel_output,
                                                  steer_output,
                                                  &safety,
                                                  0.0,
                                                  0.0,
                                                  0.0,
                                                  0.0);
    assert(!telem.traction.drift_mode);
    assert(telem.low_speed_engaged);

    safety.set_drift_enabled(true);
    safety.apply(state, 0.1, true);
    telem = vt::compute_simulation_telemetry(vsim::ModelType::MB,
                                             params,
                                             state,
                                             accel_output,
                                             wheel_output,
                                             steer_output,
                                             &safety,
                                             0.0,
                                             0.0,
                                             0.0,
                                             0.0);
    assert(telem.traction.drift_mode);
    assert(telem.low_speed_engaged);

    safety.apply(state, 2.0, true);
    telem = vt::compute_simulation_telemetry(vsim::ModelType::MB,
                                             params,
                                             state,
                                             accel_output,
                                             wheel_output,
                                             steer_output,
                                             &safety,
                                             0.0,
                                             0.0,
                                             0.0,
                                             0.0);
    assert(telem.traction.drift_mode);
    assert(!telem.low_speed_engaged);

    safety.set_drift_enabled(false);
    telem = vt::compute_simulation_telemetry(vsim::ModelType::MB,
                                             params,
                                             state,
                                             accel_output,
                                             wheel_output,
                                             steer_output,
                                             &safety,
                                             0.0,
                                             0.0,
                                             0.0,
                                             0.0);
    assert(!telem.traction.drift_mode);
    assert(!telem.low_speed_engaged);

    return 0;
}
