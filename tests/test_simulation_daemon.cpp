#include <cassert>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "common/errors.hpp"
#include "simulation/simulation_daemon.hpp"

namespace vsim = velox::simulation;
namespace vt   = velox::telemetry;

struct ReferenceRow {
    double time{};
    vt::PoseTelemetry pose{};
    vt::VelocityTelemetry velocity{};
    vt::AccelerationTelemetry acceleration{};
    vt::PowertrainTelemetry powertrain{};
    vt::AxleTelemetry front{};
    vt::AxleTelemetry rear{};
    vt::DerivedTelemetry totals{};
    bool low_speed{};
};

std::vector<ReferenceRow> load_reference(const std::string& path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("reference telemetry missing: " + path);
    }

    std::vector<ReferenceRow> rows;
    std::string               header;
    std::getline(in, header);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        std::stringstream ss(line);
        ReferenceRow      row{};
        char              comma;

        ss >> row.time >> comma >> row.pose.x >> comma >> row.pose.y >> comma >> row.pose.yaw;
        ss >> comma >> row.velocity.speed >> comma >> row.velocity.longitudinal >> comma >> row.velocity.lateral >> comma
            >> row.velocity.yaw_rate >> comma >> row.velocity.global_x >> comma >> row.velocity.global_y;
        ss >> comma >> row.acceleration.longitudinal >> comma >> row.acceleration.lateral;
        ss >> comma >> row.powertrain.total_torque >> comma >> row.powertrain.drive_torque >> comma
            >> row.powertrain.regen_torque >> comma >> row.powertrain.mechanical_power >> comma
            >> row.powertrain.battery_power >> comma >> row.powertrain.soc;
        ss >> comma >> row.front.normal_force >> comma >> row.rear.normal_force;
        ss >> comma >> row.totals.distance_traveled_m >> comma >> row.totals.energy_consumed_joules >> comma
            >> row.totals.simulation_time_s;
        int low = 0;
        ss >> comma >> low;
        row.low_speed = (low != 0);
        rows.push_back(row);
    }
    return rows;
}

void test_invalid_inputs()
{
    vsim::UserInput input{};
    input.longitudinal.throttle = std::numeric_limits<double>::infinity();
    input.dt                    = 0.1;
    input.timestamp             = 0.0;
    bool threw                  = false;
    try {
        input.validate();
    } catch (const ::velox::errors::InputError& ex) {
        threw = std::string{ex.what()}.find("throttle") != std::string::npos;
    }
    assert(threw);

    std::vector<vsim::UserInput> bad_inputs;
    for (double dt : {-0.5, 0.0}) {
        vsim::UserInput bad{};
        bad.dt                    = dt;
        bad.timestamp             = 0.0;
        bad.longitudinal.throttle = 0.0;
        bad.longitudinal.brake    = 0.0;
        bad_inputs.push_back(bad);
    }
    for (const auto& bad : bad_inputs) {
        bool bad_dt_threw = false;
        try {
            bad.validate();
        } catch (const ::velox::errors::InputError& ex) {
            bad_dt_threw = std::string{ex.what()}.find("dt") != std::string::npos;
        }
        assert(bad_dt_threw);
    }

    vsim::UserInput bad_steer{};
    bad_steer.dt                    = 0.1;
    bad_steer.timestamp             = 0.0;
    bad_steer.steering_nudge        = 2.0;
    bad_steer.longitudinal.throttle = 0.1;
    bool steer_threw                = false;
    try {
        bad_steer.validate();
    } catch (const ::velox::errors::InputError& ex) {
        steer_threw = std::string{ex.what()}.find("steering_nudge") != std::string::npos;
    }
    assert(steer_threw);

    vsim::UserInput drift_toggle{};
    drift_toggle.dt                    = 0.1;
    drift_toggle.timestamp             = 0.0;
    drift_toggle.longitudinal.throttle = 0.1;
    drift_toggle.drift_toggle          = 2.0;
    bool drift_threw                   = false;
    try {
        drift_toggle.validate();
    } catch (const ::velox::errors::InputError& ex) {
        drift_threw = std::string{ex.what()}.find("drift_toggle") != std::string::npos;
    }
    assert(drift_threw);
}

void test_drift_reset_behavior()
{
    vsim::SimulationDaemon::InitParams init{};
    init.model      = vsim::ModelType::STD;
    init.vehicle_id = 1;

    vsim::SimulationDaemon daemon(init);

    daemon.set_drift_enabled(false);
    assert(!daemon.drift_enabled());
    assert(daemon.simulator());
    assert(!daemon.simulator()->safety().drift_enabled());

    vsim::UserInput input{};
    input.timestamp             = 0.0;
    input.dt                    = 0.05;
    input.longitudinal.throttle = 0.6;
    input.steering_nudge        = 0.8;

    daemon.step(input);
    assert(daemon.accel_controller());
    assert(daemon.accel_controller()->throttle() > 0.0);
    assert(std::abs(daemon.steering_wheel()->last_output().angle) > 0.0);

    vsim::ResetParams reset{};
    reset.dt = 0.1;
    daemon.reset(reset);

    assert(daemon.drift_enabled());
    assert(daemon.simulator());
    assert(daemon.simulator()->safety().drift_enabled());
    assert(daemon.accel_controller());
    assert(daemon.accel_controller()->throttle() == 0.0);
    assert(std::abs(daemon.steering_wheel()->last_output().angle) == 0.0);
    assert(std::abs(daemon.steering_controller()->last_output().angle) == 0.0);
}

void test_direct_mode_torque_and_steering_limits()
{
    vsim::SimulationDaemon::InitParams init{};
    init.model          = vsim::ModelType::ST;
    init.vehicle_id     = 1;
    init.control_mode   = vsim::ControlMode::Direct;
    init.config_root    = "config";
    init.parameter_root = "parameters";

    vsim::SimulationDaemon daemon(init);

    vsim::ResetParams reset{};
    reset.control_mode  = vsim::ControlMode::Direct;
    reset.initial_state = std::vector<double>(7, 0.0);
    reset.dt            = 0.05;
    daemon.reset(reset);

    assert(daemon.control_mode() == vsim::ControlMode::Direct);
    assert(daemon.steering_controller());
    const double max_angle = daemon.steering_controller()->max_angle();

    vsim::UserInput over_limit{};
    over_limit.control_mode  = vsim::ControlMode::Direct;
    over_limit.timestamp     = 0.0;
    over_limit.dt            = 0.05;
    over_limit.steering_angle = max_angle * 2.0;
    over_limit.axle_torques  = {0.0};
    bool rejected            = false;
    try {
        (void)daemon.step(over_limit);
    } catch (const ::velox::errors::InputError&) {
        rejected = true;
    }
    assert(rejected);

    vsim::UserInput direct{};
    direct.control_mode   = vsim::ControlMode::Direct;
    direct.timestamp      = 0.0;
    direct.dt             = 0.05;
    direct.steering_angle = max_angle;
    const double commanded_torque = 200.0;
    direct.axle_torques           = {commanded_torque};

    const auto telemetry = daemon.step(direct);
    assert(telemetry.steering.desired_angle <= max_angle + 1e-9);
    assert(telemetry.steering.actual_angle <= max_angle + 1e-9);
    assert(std::abs(telemetry.powertrain.drive_torque - commanded_torque) < 1e-6);
    assert(std::abs(telemetry.powertrain.total_torque - commanded_torque) < 1e-6);
    assert(std::abs(telemetry.controller.acceleration) > 0.0);
}

void test_mode_switch_does_not_leak_state()
{
    vsim::SimulationDaemon::InitParams init{};
    init.model          = vsim::ModelType::ST;
    init.vehicle_id     = 1;
    init.control_mode   = vsim::ControlMode::Keyboard;
    init.config_root    = "config";
    init.parameter_root = "parameters";

    vsim::SimulationDaemon daemon(init);

    vsim::ResetParams keyboard_reset{};
    keyboard_reset.control_mode  = vsim::ControlMode::Keyboard;
    keyboard_reset.initial_state = std::vector<double>(7, 0.0);
    keyboard_reset.dt            = 0.1;
    daemon.reset(keyboard_reset);

    vsim::UserInput keyboard_input{};
    keyboard_input.timestamp             = 0.0;
    keyboard_input.dt                    = 0.1;
    keyboard_input.longitudinal.throttle = 0.6;
    keyboard_input.longitudinal.brake    = 0.0;
    keyboard_input.steering_nudge        = 0.2;

    const auto keyboard_telem = daemon.step(keyboard_input);
    assert(keyboard_telem.controller.throttle > 0.0);
    assert(std::abs(keyboard_telem.steering.desired_angle) > 0.0);

    vsim::ResetParams direct_reset{};
    direct_reset.control_mode  = vsim::ControlMode::Direct;
    direct_reset.initial_state = std::vector<double>(7, 0.0);
    direct_reset.dt            = 0.05;
    daemon.reset(direct_reset);

    assert(daemon.control_mode() == vsim::ControlMode::Direct);

    vsim::UserInput direct{};
    direct.control_mode   = vsim::ControlMode::Direct;
    direct.timestamp      = 0.0;
    direct.dt             = 0.05;
    direct.steering_angle = daemon.steering_controller()->max_angle() * 0.5;
    direct.axle_torques   = {120.0};

    const auto direct_telem = daemon.step(direct);
    assert(direct_telem.controller.throttle == 0.0);
    assert(direct_telem.powertrain.total_torque > 0.0);
    assert(direct_telem.steering.desired_angle > 0.0);

    vsim::ResetParams coasting_reset{};
    coasting_reset.control_mode  = vsim::ControlMode::Keyboard;
    coasting_reset.initial_state = std::vector<double>(7, 0.0);
    coasting_reset.dt            = 0.1;
    daemon.reset(coasting_reset);

    assert(daemon.control_mode() == vsim::ControlMode::Keyboard);

    vsim::UserInput coasting{};
    coasting.timestamp             = 0.0;
    coasting.dt                    = 0.1;
    coasting.longitudinal.throttle = 0.0;
    coasting.longitudinal.brake    = 0.0;
    coasting.steering_nudge        = 0.0;

    const auto coasting_telem = daemon.step(coasting);
    assert(std::abs(coasting_telem.controller.drive_force) < 1e-9);
    assert(std::abs(coasting_telem.powertrain.total_torque) < 1e-9);
    assert(std::abs(coasting_telem.steering.actual_angle) <= daemon.steering_controller()->max_angle());
}

void compare_row(const ReferenceRow& ref, const vt::SimulationTelemetry& telem)
{
    auto close = [](double a, double b, double tol = 1e-6) { return std::abs(a - b) <= tol; };
    assert(close(ref.pose.x, telem.pose.x));
    assert(close(ref.pose.y, telem.pose.y));
    assert(close(ref.pose.yaw, telem.pose.yaw));

    assert(close(ref.velocity.speed, telem.velocity.speed));
    assert(close(ref.velocity.longitudinal, telem.velocity.longitudinal));
    assert(close(ref.velocity.yaw_rate, telem.velocity.yaw_rate));
    assert(close(ref.velocity.global_x, telem.velocity.global_x));
    assert(close(ref.velocity.global_y, telem.velocity.global_y));

    assert(close(ref.acceleration.longitudinal, telem.acceleration.longitudinal));
    assert(close(ref.powertrain.soc, telem.powertrain.soc));
    assert(close(ref.front.normal_force, telem.front_axle.normal_force));
    assert(close(ref.rear.normal_force, telem.rear_axle.normal_force));
    assert(close(ref.totals.simulation_time_s, telem.totals.simulation_time_s));
    assert(ref.low_speed == telem.low_speed_engaged);
}

int main()
{
    test_invalid_inputs();
    test_drift_reset_behavior();
    test_direct_mode_torque_and_steering_limits();
    test_mode_switch_does_not_leak_state();

    const auto reference = load_reference("tests/output/zero_input_reference.csv");
    if (reference.empty()) {
        std::cerr << "No reference telemetry rows loaded" << '\n';
        return 1;
    }

    vsim::SimulationDaemon::InitParams init{};
    init.model = vsim::ModelType::ST;
    init.vehicle_id = 1;

    vsim::SimulationDaemon daemon(init);

    vsim::ResetParams reset{};
    reset.model        = vsim::ModelType::ST;
    reset.vehicle_id   = 1;
    reset.initial_state = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    reset.dt           = 0.1;
    daemon.reset(reset);

    std::vector<vt::SimulationTelemetry> captured;
    double                               timestamp = 0.0;
    for (std::size_t i = 0; i < reference.size(); ++i) {
        vsim::UserInput input{};
        input.timestamp             = timestamp;
        input.dt                    = 0.1;
        input.longitudinal.throttle = 0.0;
        input.longitudinal.brake    = 0.0;
        input.steering_nudge        = 0.0;
        captured.push_back(daemon.step(input));
        timestamp += input.dt;
    }

    assert(captured.size() == reference.size());
    for (std::size_t i = 0; i < captured.size(); ++i) {
        compare_row(reference[i], captured[i]);
    }

    std::cout << "Simulation daemon telemetry matched reference for " << captured.size() << " samples\n";
    return 0;
}
