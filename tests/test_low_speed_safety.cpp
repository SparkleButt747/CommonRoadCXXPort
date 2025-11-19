#include <cassert>
#include <cmath>
#include <exception>
#include <iostream>
#include <optional>
#include <vector>

#include "simulation/low_speed_safety.hpp"
#include "io/config_manager.hpp"
#include "simulation/vehicle_simulator.hpp"
#include "vehicle_parameters.hpp"

namespace vsim = velox::simulation;
namespace vio  = velox::io;

vsim::LowSpeedSafetyConfig make_default_config()
{
    vsim::LowSpeedSafetyConfig cfg{};
    cfg.normal.engage_speed     = 0.4;
    cfg.normal.release_speed    = 0.8;
    cfg.normal.yaw_rate_limit   = 0.5;
    cfg.normal.slip_angle_limit = 0.35;

    cfg.drift.engage_speed     = 0.2;
    cfg.drift.release_speed    = 1.0;
    cfg.drift.yaw_rate_limit   = 0.8;
    cfg.drift.slip_angle_limit = 0.6;

    cfg.stop_speed_epsilon = 0.05;
    cfg.drift_enabled      = false;
    return cfg;
}

void test_wheel_speed_clamp()
{
    auto cfg = make_default_config();

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
    assert(std::abs(state[5]) <= cfg.normal.yaw_rate_limit + 1e-9);
}

void test_latch_release_thresholds()
{
    auto cfg = make_default_config();

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
    safety.apply(state, cfg.normal.engage_speed - 1e-3, true);
    assert(safety.engaged());
    state[5] = cfg.normal.yaw_rate_limit * 2.0;

    safety.apply(state, cfg.normal.release_speed - 5e-4, true);
    assert(safety.engaged());

    safety.apply(state, cfg.normal.release_speed + 1e-3, false);
    assert(safety.engaged());

    safety.apply(state, cfg.normal.release_speed + 1e-3, true);
    assert(!safety.engaged());
    assert(std::abs(state[5]) <= cfg.normal.yaw_rate_limit + 1e-9);
}

void test_drift_profile_selection()
{
    auto cfg = make_default_config();
    cfg.drift_enabled       = false;
    cfg.drift.yaw_rate_limit = cfg.normal.yaw_rate_limit * 2.0;

    vsim::LowSpeedSafety safety(cfg,
                                /*longitudinal_index=*/std::nullopt,
                                /*lateral_index=*/std::nullopt,
                                /*yaw_rate_index=*/1,
                                /*slip_index=*/std::nullopt,
                                /*wheel_speed_indices=*/{},
                                /*steering_index=*/std::nullopt,
                                /*wheelbase=*/std::nullopt,
                                /*rear_length=*/std::nullopt);

    std::vector<double> state{0.0, cfg.drift.yaw_rate_limit * 1.5};
    safety.apply(state, cfg.normal.engage_speed - 1e-3, true);
    assert(safety.engaged());
    assert(std::abs(state[1]) <= cfg.normal.yaw_rate_limit + 1e-9);

    safety.set_drift_enabled(true);
    state[1] = cfg.drift.yaw_rate_limit * 1.5;
    safety.apply(state, cfg.drift.engage_speed - 1e-3, true);
    assert(safety.engaged());
    assert(std::abs(state[1]) <= cfg.drift.yaw_rate_limit + 1e-9);
}

void test_drift_mode_relaxes_unlatched_clamp()
{
    auto cfg = make_default_config();
    cfg.drift_enabled = true;

    velox::models::VehicleParameters params{};
    params.a = 1.4;
    params.b = 1.3;

    vsim::LowSpeedSafety safety(
        cfg,
        /*longitudinal_index=*/3,
        /*lateral_index=*/std::nullopt,
        /*yaw_rate_index=*/5,
        /*slip_index=*/6,
        /*wheel_speed_indices=*/{},
        /*steering_index=*/2,
        /*wheelbase=*/params.a + params.b,
        /*rear_length=*/params.b);

    safety.set_drift_enabled(true);

    const double yaw_limit  = cfg.drift.yaw_rate_limit;
    const double slip_limit = cfg.drift.slip_angle_limit;
    const double steering   = 0.2;

    std::vector<double> state(9, 0.0);
    state[2] = steering;
    state[5] = yaw_limit * 1.5;
    state[6] = slip_limit * 1.2;

    const double transition_band = cfg.drift.release_speed - cfg.drift.engage_speed;
    const double cruising_speed  = cfg.drift.release_speed + transition_band + 0.2;
    safety.apply(state, cruising_speed, true);

    assert(!safety.engaged());
    assert(state[5] > yaw_limit);
    assert(state[6] > slip_limit);

    state[5] = yaw_limit * 1.5;
    state[6] = slip_limit * 1.2;

    const double crawl_speed = cfg.drift.engage_speed - 1e-3;
    safety.apply(state, crawl_speed, true);

    assert(safety.engaged());
    assert(std::abs(state[5]) <= 1e-9);
    assert(std::abs(state[6]) <= slip_limit + 1e-9);
}

void test_slip_latch_tracks_velocity_heading()
{
    auto cfg = make_default_config();
    cfg.drift_enabled = false;

    vsim::LowSpeedSafety safety(
        cfg,
        /*longitudinal_index=*/3,
        /*lateral_index=*/std::nullopt,
        /*yaw_rate_index=*/5,
        /*slip_index=*/6,
        /*wheel_speed_indices=*/{},
        /*steering_index=*/2,
        /*wheelbase=*/std::make_optional(2.6),
        /*rear_length=*/std::make_optional(1.3));

    std::vector<double> state(9, 0.0);
    state[2] = 0.1;
    state[3] = cfg.normal.engage_speed * 0.9;
    state[5] = 0.2;
    state[6] = 0.25; // below slip limit so it should be preserved when latched

    safety.apply(state, cfg.normal.engage_speed * 0.9, true);
    assert(safety.engaged());
    assert(std::abs(state[6] - 0.25) <= 1e-6);
}

void test_transition_blend_preloads_clamp()
{
    auto cfg = make_default_config();
    cfg.drift_enabled = true;

    vsim::LowSpeedSafety safety(
        cfg,
        /*longitudinal_index=*/3,
        /*lateral_index=*/std::nullopt,
        /*yaw_rate_index=*/5,
        /*slip_index=*/6,
        /*wheel_speed_indices=*/{},
        /*steering_index=*/2,
        /*wheelbase=*/std::make_optional(2.7),
        /*rear_length=*/std::make_optional(1.3));

    safety.set_drift_enabled(true);

    const double yaw_limit  = cfg.drift.yaw_rate_limit;
    const double slip_limit = cfg.drift.slip_angle_limit;
    const double transition_speed = cfg.drift.release_speed + (cfg.drift.release_speed - cfg.drift.engage_speed) * 0.9;

    std::vector<double> state(9, 0.0);
    state[2] = 0.25;
    state[5] = yaw_limit * 1.5;
    state[6] = slip_limit * 1.5;

    safety.apply(state, transition_speed, true);
    assert(!safety.engaged());
    assert(std::abs(state[5]) < yaw_limit * 1.5);
    assert(std::abs(state[6]) < slip_limit * 1.5);

    state[5] = yaw_limit * 1.2;
    state[6] = slip_limit * 1.2;
    safety.apply(state, cfg.drift.release_speed + 1e-4, true);
    assert(std::abs(state[5]) <= yaw_limit + 1e-6);
    assert(std::abs(state[6]) <= slip_limit + 1e-6);
}

void test_vehicle_simulator_stop()
{
    auto cfg = make_default_config();

    vsim::ModelInterface model{};
    model.init_fn = [](const std::vector<double>& state,
                       const velox::models::VehicleParameters&) {
        return state;
    };
    model.dynamics_fn = [](const std::vector<double>& state,
                           const std::vector<double>& control,
                           const velox::models::VehicleParameters&,
                           double) {
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
                        const velox::models::VehicleParameters&) {
        return (state.size() > 3) ? std::abs(state[3]) : 0.0;
    };

    velox::models::VehicleParameters params{};
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
    simulator.reset(std::vector<double>{0.0, 0.0, 0.0, 0.5});

    for (int i = 0; i < 400; ++i) {
        simulator.step(std::vector<double>{0.0, -5.0});
    }
    const auto& stopped = simulator.state();
    assert(stopped[3] >= -1e-6);

    double previous_speed = simulator.speed();
    bool accelerated = false;
    for (int i = 0; i < 200; ++i) {
        simulator.step(std::vector<double>{0.0, 2.0});
        const double current_speed = simulator.speed();
        if (current_speed > previous_speed + 1e-3) {
            accelerated = true;
            break;
        }
        previous_speed = current_speed;
    }
    assert(accelerated);
}

void test_rk4_predictor_does_not_latch_above_engage()
{
    auto cfg = make_default_config();

    vsim::LowSpeedSafety safety(
        cfg,
        /*longitudinal_index=*/0,
        /*lateral_index=*/std::nullopt,
        /*yaw_rate_index=*/1,
        /*slip_index=*/2,
        /*wheel_speed_indices=*/{},
        /*steering_index=*/std::nullopt,
        /*wheelbase=*/std::nullopt,
        /*rear_length=*/std::nullopt);

    const double target_speed = cfg.normal.engage_speed + 0.005;
    const double rate         = 0.1;

    vsim::ModelInterface model{};
    model.init_fn = [](const std::vector<double>& state,
                       const velox::models::VehicleParameters&) {
        return state;
    };
    model.dynamics_fn = [target_speed, rate](const std::vector<double>& state,
                                            const std::vector<double>&,
                                            const velox::models::VehicleParameters&,
                                            double) {
        std::vector<double> rhs(state.size(), 0.0);
        if (!state.empty()) {
            const double speed = state[0];
            rhs[0] = (speed > target_speed) ? -rate : rate;
        }
        return rhs;
    };
    model.speed_fn = [](const std::vector<double>& state,
                        const velox::models::VehicleParameters&) {
        return state.empty() ? 0.0 : state[0];
    };

    velox::models::VehicleParameters params{};

    vsim::VehicleSimulator simulator(std::move(model), params, 0.2, std::move(safety));
    const std::vector<double> initial_state{target_speed + 0.002, 0.2, 0.1};
    simulator.reset(initial_state);

    const auto& state = simulator.step(std::vector<double>{0.0, 0.0});
    const double realized_speed = simulator.speed();

    assert(realized_speed > cfg.normal.engage_speed);
    assert(!simulator.safety().engaged());
    assert(std::abs(state[1] - initial_state[1]) < 1e-12);
    assert(std::abs(state[2] - initial_state[2]) < 1e-12);
    assert(state[1] > 0.0);
    assert(state[2] > 0.0);
}

void test_std_predictor_wheel_speeds_zeroed_when_engaged()
{
    auto cfg = make_default_config();

    velox::models::VehicleParameters params{};
    params.a = 1.4;
    params.b = 1.3;

    vsim::LowSpeedSafety safety(
        cfg,
        /*longitudinal_index=*/3,
        /*lateral_index=*/std::nullopt,
        /*yaw_rate_index=*/std::nullopt,
        /*slip_index=*/std::nullopt,
        /*wheel_speed_indices=*/{7, 8},
        /*steering_index=*/2,
        /*wheelbase=*/params.a + params.b,
        /*rear_length=*/params.b);

    std::vector<double> state(9, 0.0);
    state[7] = 0.04;
    state[8] = 0.02;

    const double transition_band  = cfg.normal.release_speed - cfg.normal.engage_speed;
    const double disengaged_speed = cfg.normal.release_speed + transition_band + 0.05;
    safety.apply(state, disengaged_speed, true);
    assert(!safety.engaged());
    assert(state[7] > 0.0);
    assert(state[8] > 0.0);

    const double engaged_speed = cfg.normal.engage_speed - 1e-3;
    safety.apply(state, engaged_speed, true);
    assert(safety.engaged());
    assert(state[7] == 0.0);
    assert(state[8] == 0.0);

    auto ensure_predictor_zero = [&](double v7, double v8) {
        state[7] = v7;
        state[8] = v8;
        safety.apply(state, engaged_speed, false);
        assert(state[7] == 0.0);
        assert(state[8] == 0.0);
    };

    ensure_predictor_zero(0.03, 0.01);
    ensure_predictor_zero(0.02, 0.02);
    ensure_predictor_zero(0.01, 0.04);
}

void test_mb_predictor_wheel_speeds_zeroed_when_engaged()
{
    auto cfg = make_default_config();

    velox::models::VehicleParameters params{};
    params.a = 1.4;
    params.b = 1.3;

    vsim::LowSpeedSafety safety(
        cfg,
        /*longitudinal_index=*/3,
        /*lateral_index=*/10,
        /*yaw_rate_index=*/std::nullopt,
        /*slip_index=*/std::nullopt,
        /*wheel_speed_indices=*/{23, 24, 25, 26},
        /*steering_index=*/2,
        /*wheelbase=*/params.a + params.b,
        /*rear_length=*/params.b);

    std::vector<double> state(27, 0.0);
    state[23] = 0.04;
    state[24] = 0.03;
    state[25] = 0.02;
    state[26] = 0.01;

    const double transition_band  = cfg.normal.release_speed - cfg.normal.engage_speed;
    const double disengaged_speed = cfg.normal.release_speed + transition_band + 0.05;
    safety.apply(state, disengaged_speed, true);
    assert(!safety.engaged());
    assert(state[23] > 0.0);
    assert(state[24] > 0.0);
    assert(state[25] > 0.0);
    assert(state[26] > 0.0);

    const double engaged_speed = cfg.normal.engage_speed - 1e-3;
    safety.apply(state, engaged_speed, true);
    assert(safety.engaged());
    for (int idx : {23, 24, 25, 26}) {
        assert(state[idx] == 0.0);
    }

    auto ensure_predictor_zero = [&](double v23, double v24, double v25, double v26) {
        state[23] = v23;
        state[24] = v24;
        state[25] = v25;
        state[26] = v26;
        safety.apply(state, engaged_speed, false);
        for (int idx : {23, 24, 25, 26}) {
            assert(state[idx] == 0.0);
        }
    };

    ensure_predictor_zero(0.03, 0.02, 0.01, 0.01);
    ensure_predictor_zero(0.02, 0.02, 0.02, 0.02);
    ensure_predictor_zero(0.01, 0.03, 0.04, 0.01);
}

void test_model_specific_config_loading()
{
    vio::ConfigManager configs{};
    const auto default_cfg = configs.load_low_speed_safety_config(vsim::ModelType::ST);
    const auto std_cfg = configs.load_low_speed_safety_config(vsim::ModelType::STD);
    const auto mb_cfg  = configs.load_low_speed_safety_config(vsim::ModelType::MB);
    const auto st_cfg  = configs.load_low_speed_safety_config(vsim::ModelType::ST);

    assert(std::abs(std_cfg.normal.release_speed - 0.8) < 1e-9);
    assert(std::abs(std_cfg.normal.engage_speed - 0.4) < 1e-9);
    assert(std::abs(mb_cfg.normal.release_speed - 0.6) < 1e-9);
    assert(std::abs(mb_cfg.normal.engage_speed - 0.35) < 1e-9);
    assert(std::abs(mb_cfg.normal.yaw_rate_limit - 0.7) < 1e-9);
    assert(std::abs(mb_cfg.normal.slip_angle_limit - 0.5) < 1e-9);
    assert(std::abs(st_cfg.normal.release_speed - default_cfg.normal.release_speed) < 1e-9);
    assert(!default_cfg.drift_enabled);
    assert(std_cfg.drift_enabled);
    assert(!mb_cfg.drift_enabled);
}

void test_mb_latch_releases_after_acceleration()
{
    vio::ConfigManager configs{};
    const auto cfg = configs.load_low_speed_safety_config(vsim::ModelType::MB);
    vsim::LowSpeedSafety safety(
        cfg,
        /*longitudinal_index=*/0,
        /*lateral_index=*/std::nullopt,
        /*yaw_rate_index=*/std::nullopt,
        /*slip_index=*/std::nullopt,
        /*wheel_speed_indices=*/{},
        /*steering_index=*/std::nullopt,
        /*wheelbase=*/std::nullopt,
        /*rear_length=*/std::nullopt);

    vsim::ModelInterface model{};
    model.init_fn = [](const std::vector<double>& state,
                       const velox::models::VehicleParameters&) {
        return state;
    };
    model.dynamics_fn = [](const std::vector<double>& state,
                           const std::vector<double>& control,
                           const velox::models::VehicleParameters&,
                           double) {
        std::vector<double> rhs(state.size(), 0.0);
        if (!state.empty()) {
            rhs[0] = (control.size() > 1) ? control[1] : 0.0;
        }
        return rhs;
    };
    model.speed_fn = [](const std::vector<double>& state,
                        const velox::models::VehicleParameters&) {
        if (state.empty()) {
            return 0.0;
        }
        return std::fabs(state[0]);
    };

    velox::models::VehicleParameters params{};
    vsim::VehicleSimulator simulator(std::move(model), params, 0.005, std::move(safety));
    simulator.reset(std::vector<double>{0.0});

    assert(simulator.safety().engaged());

    bool released = false;
    for (int step = 0; step < 400; ++step) {
        simulator.step(std::vector<double>{0.0, 2.5});
        if (!simulator.safety().engaged() && simulator.speed() >= cfg.normal.release_speed) {
            released = true;
            break;
        }
    }
    assert(released);
}

int main()
{
    try {
        test_wheel_speed_clamp();
        test_latch_release_thresholds();
        test_drift_profile_selection();
        test_drift_mode_relaxes_unlatched_clamp();
        test_slip_latch_tracks_velocity_heading();
        test_transition_blend_preloads_clamp();
        test_vehicle_simulator_stop();
        test_rk4_predictor_does_not_latch_above_engage();
        test_std_predictor_wheel_speeds_zeroed_when_engaged();
        test_mb_predictor_wheel_speeds_zeroed_when_engaged();
        test_model_specific_config_loading();
        test_mb_latch_releases_after_acceleration();
    } catch (const std::exception& ex) {
        std::cerr << "Test raised exception: " << ex.what() << '\n';
        return 1;
    }
    std::cout << "All low-speed safety tests passed\n";
    return 0;
}
