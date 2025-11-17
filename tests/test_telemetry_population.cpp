#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "controllers/longitudinal/final_accel_controller.hpp"
#include "controllers/steering_controller.hpp"
#include "simulation/low_speed_safety.hpp"
#include "telemetry/telemetry.hpp"
#include "vehicle/parameters_vehicle1.hpp"

namespace vt = velox::telemetry;
namespace vml = velox::controllers::longitudinal;
namespace vs  = velox::controllers;
namespace vsim = velox::simulation;
namespace vm = velox::models;

int main()
{
    const auto params = vm::parameters_vehicle1();

    vml::ControllerOutput accel_output{};
    accel_output.acceleration = 1.25;
    accel_output.throttle     = 0.4;
    accel_output.brake        = 0.0;
    accel_output.drive_force  = 800.0;
    accel_output.brake_force  = 0.0;
    accel_output.regen_force  = 10.0;
    accel_output.hydraulic_force = 0.0;
    accel_output.drag_force      = 5.0;
    accel_output.rolling_force   = 3.0;
    accel_output.mechanical_power = 1200.0;
    accel_output.battery_power    = 800.0;
    accel_output.soc              = 0.68;

    vs::SteeringWheel::Output steering_wheel_output{};
    steering_wheel_output.target_angle = 0.12;
    steering_wheel_output.rate         = 0.8;

    vs::FinalSteerController::Output steer_output{};
    steer_output.angle = 0.1;
    steer_output.rate  = 0.75;

    std::vector<double> state{1.0, 2.0, 0.1, 6.0, 0.25, 0.0, 0.05};
    const double        measured_speed = 0.0; // use state-derived speed

    vt::SimulationTelemetry telemetry = vt::compute_simulation_telemetry(
        vsim::ModelType::ST,
        params,
        state,
        accel_output,
        steering_wheel_output,
        steer_output,
        nullptr,
        measured_speed,
        /*distance*/ 12.0,
        /*energy*/ 34.0,
        /*time*/ 2.5);

    assert(std::abs(telemetry.pose.x - 1.0) < 1e-9);
    assert(std::abs(telemetry.pose.y - 2.0) < 1e-9);
    assert(std::abs(telemetry.pose.yaw - 0.25) < 1e-9);

    const double expected_speed = 6.0;
    assert(std::abs(telemetry.velocity.speed - expected_speed) < 1e-9);
    const double expected_heading = 0.25 + 0.05;
    assert(std::abs(telemetry.velocity.global_x - expected_speed * std::cos(expected_heading)) < 1e-9);
    assert(std::abs(telemetry.velocity.global_y - expected_speed * std::sin(expected_heading)) < 1e-9);

    const double wheelbase = params.a + params.b;
    const double expected_yaw_rate = expected_speed * std::tan(state[2]) / wheelbase;
    assert(std::abs(telemetry.velocity.yaw_rate - expected_yaw_rate) < 1e-9);
    const double expected_lat_accel = expected_speed * expected_speed * std::tan(state[2]) / wheelbase;
    assert(std::abs(telemetry.acceleration.lateral - expected_lat_accel) < 1e-9);
    assert(std::abs(telemetry.acceleration.longitudinal - accel_output.acceleration) < 1e-9);

    assert(std::abs(telemetry.controller.throttle - 0.4) < 1e-9);
    assert(std::abs(telemetry.controller.regen_force - 10.0) < 1e-9);
    assert(std::abs(telemetry.powertrain.drive_torque - accel_output.drive_force * params.R_w) < 1e-6);
    assert(std::abs(telemetry.powertrain.soc - accel_output.soc) < 1e-12);

    const double front_normal = params.m * 9.81 * (params.b / wheelbase);
    const double rear_normal  = params.m * 9.81 * (params.a / wheelbase);
    assert(std::abs(telemetry.front_axle.normal_force - front_normal) < 1e-6);
    assert(std::abs(telemetry.rear_axle.normal_force - rear_normal) < 1e-6);

    const double expected_front_left_speed = (state[3] - telemetry.velocity.yaw_rate * (0.5 * params.T_f))
        * std::cos(state[2]);
    assert(std::abs(telemetry.front_axle.left.speed - expected_front_left_speed) < 1e-6);

    assert(std::abs(telemetry.totals.distance_traveled_m - 12.0) < 1e-9);
    assert(std::abs(telemetry.totals.energy_consumed_joules - 34.0) < 1e-9);
    assert(std::abs(telemetry.totals.simulation_time_s - 2.5) < 1e-9);

    std::cout << vt::to_json(telemetry) << '\n';
    return 0;
}
