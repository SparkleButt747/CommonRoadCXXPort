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
