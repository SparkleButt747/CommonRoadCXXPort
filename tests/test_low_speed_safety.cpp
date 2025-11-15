#include <cassert>
#include <cmath>
#include <exception>
#include <iostream>
#include <optional>
#include <vector>

#include "sim/low_speed_safety.hpp"
#include "sim/vehicle_simulator.hpp"
#include "vehicle_parameters.hpp"

namespace vsim = vehiclemodels::sim;

void test_wheel_speed_clamp()
{
    vsim::LowSpeedSafetyConfig cfg{};
    cfg.engage_speed        = 0.4;
    cfg.release_speed       = 0.8;
    cfg.yaw_rate_limit      = 0.5;
    cfg.slip_angle_limit    = 0.35;
    cfg.stop_speed_epsilon  = 0.05;

    vsim::LowSpeedSafety safety(
        cfg,
        /*longitudinal_index=*/3,
        /*lateral_index=*/std::nullopt,
        /*yaw_rate_index=*/5,
        /*slip_index=*/6,
        /*wheel_speed_indices=*/{7, 8},
        /*steering_index=*/2,
        /*wheelbase=*/std::make_optional(2.8),
        /*rear_length=*/std::make_optional(1.3)
    );

    std::vector<double> state{0.0, 0.0, 0.05, 0.02, 0.0, 0.3, 0.1, -3.0, 0.02};
    safety.apply(state, 0.02, true);

    assert(safety.engaged());
    assert(state[3] >= 0.0);
    assert(state[7] == 0.0);
    assert(std::abs(state[5]) <= cfg.yaw_rate_limit + 1e-9);
}

void test_latch_release()
{
    vsim::LowSpeedSafetyConfig cfg{};
    cfg.engage_speed        = 0.4;
    cfg.release_speed       = 0.8;
    cfg.yaw_rate_limit      = 0.5;
    cfg.slip_angle_limit    = 0.35;
    cfg.stop_speed_epsilon  = 0.05;

    vsim::LowSpeedSafety safety(
        cfg,
        /*longitudinal_index=*/3,
        /*lateral_index=*/std::nullopt,
        /*yaw_rate_index=*/5,
        /*slip_index=*/std::nullopt,
        /*wheel_speed_indices=*/{},
        /*steering_index=*/2,
        /*wheelbase=*/std::make_optional(2.8),
        /*rear_length=*/std::make_optional(1.3)
    );

    std::vector<double> state{0.0, 0.0, 0.02, 0.05, 0.0, 0.4};
    safety.apply(state, 0.05, true);
    assert(safety.engaged());
    safety.apply(state, 1.0, true);
    assert(!safety.engaged());
    assert(std::abs(state[5]) <= cfg.yaw_rate_limit + 1e-9);
}

void test_vehicle_simulator_stop()
{
    vsim::LowSpeedSafetyConfig cfg{};
    cfg.engage_speed        = 0.4;
    cfg.release_speed       = 0.8;
    cfg.yaw_rate_limit      = 0.5;
    cfg.slip_angle_limit    = 0.35;
    cfg.stop_speed_epsilon  = 0.05;

    vsim::ModelInterface model{};
    model.dynamics_fn = [](const std::vector<double>& state,
                           const std::vector<double>& control,
                           const vehiclemodels::VehicleParameters&) {
        std::vector<double> rhs(state.size(), 0.0);
        if (state.size() > 0) {
            rhs[0] = (state.size() > 3) ? state[3] : 0.0;
        }
        if (state.size() > 2) {
            rhs[2] = (control.size() > 0) ? control[0] : 0.0;
        }
        if (state.size() > 3) {
            rhs[3] = (control.size() > 1) ? control[1] : 0.0;
        }
        return rhs;
    };
    model.speed_fn = [](const std::vector<double>& state,
                        const vehiclemodels::VehicleParameters&) {
        return (state.size() > 3) ? std::abs(state[3]) : 0.0;
    };

    vehiclemodels::VehicleParameters params{};
    params.a  = 1.4;
    params.b  = 1.3;
    params.R_w = 0.3;
    params.m   = 1200.0;

    auto safety = vsim::LowSpeedSafety(
        cfg,
        /*longitudinal_index=*/3,
        /*lateral_index=*/std::nullopt,
        /*yaw_rate_index=*/std::nullopt,
        /*slip_index=*/std::nullopt,
        /*wheel_speed_indices=*/{},
        /*steering_index=*/2,
        /*wheelbase=*/std::make_optional(params.a + params.b),
        /*rear_length=*/std::make_optional(params.b)
    );

    vsim::VehicleSimulator simulator(std::move(model), params, 0.01, std::move(safety));
    simulator.reset({0.0, 0.0, 0.0, 0.5});

    for (int i = 0; i < 400; ++i) {
        simulator.step({0.0, -5.0});
    }
    const auto& stopped = simulator.state();
    assert(stopped[3] >= -1e-6);

    double previous_speed = simulator.speed();
    bool accelerated = false;
    for (int i = 0; i < 200; ++i) {
        simulator.step({0.0, 2.0});
        const double current_speed = simulator.speed();
        if (current_speed > previous_speed + 1e-3) {
            accelerated = true;
            break;
        }
        previous_speed = current_speed;
    }
    assert(accelerated);
}

int main()
{
    try {
        test_wheel_speed_clamp();
        test_latch_release();
        test_vehicle_simulator_stop();
    } catch (const std::exception& ex) {
        std::cerr << "Test raised exception: " << ex.what() << '\n';
        return 1;
    }
    std::cout << "All low-speed safety tests passed\n";
    return 0;
}
