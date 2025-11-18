#include <cassert>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "io/config_manager.hpp"
#include "simulation/model_timing.hpp"
#include "simulation/vehicle_simulator.hpp"
#include "vehicle/parameters_vehicle1.hpp"
#include "models/vehiclemodels/init_mb.hpp"
#include "models/vehiclemodels/init_std.hpp"
#include "models/vehiclemodels/vehicle_dynamics_mb.hpp"
#include "models/vehiclemodels/vehicle_dynamics_std.hpp"

namespace vm   = velox::models;
namespace vsim = velox::simulation;
namespace vio  = velox::io;

static vsim::ModelInterface build_std_interface()
{
    vsim::ModelInterface iface{};
    iface.init_fn = [](const std::vector<double>& init_state,
                       const vm::VehicleParameters& params) {
        return vm::init_std(init_state, params);
    };
    iface.dynamics_fn = [](const std::vector<double>& x,
                           const std::vector<double>& u,
                           const vm::VehicleParameters& params,
                           double dt) {
        return vm::vehicle_dynamics_std(x, u, params, dt);
    };
    iface.speed_fn = [](const std::vector<double>& state,
                        const vm::VehicleParameters&) {
        return (state.size() > 3) ? std::abs(state[3]) : 0.0;
    };
    return iface;
}

static vsim::ModelInterface build_mb_interface()
{
    vsim::ModelInterface iface{};
    iface.init_fn = [](const std::vector<double>& init_state,
                       const vm::VehicleParameters& params) {
        return vm::init_mb(init_state, params);
    };
    iface.dynamics_fn = [](const std::vector<double>& x,
                           const std::vector<double>& u,
                           const vm::VehicleParameters& params,
                           double) {
        return vm::vehicle_dynamics_mb(x, u, params);
    };
    iface.speed_fn = [](const std::vector<double>& state,
                        const vm::VehicleParameters&) {
        if (state.size() > 10) {
            return std::hypot(state[3], state[10]);
        }
        if (state.size() > 3) {
            return std::abs(state[3]);
        }
        return 0.0;
    };
    return iface;
}

static vsim::LowSpeedSafety make_std_safety(const vm::VehicleParameters& params,
                                            const vsim::LowSpeedSafetyConfig& cfg)
{
    return vsim::LowSpeedSafety(cfg,
                                /*longitudinal_index=*/3,
                                /*lateral_index=*/std::nullopt,
                                /*yaw_rate_index=*/5,
                                /*slip_index=*/6,
                                /*wheel_speed_indices=*/{7, 8},
                                /*steering_index=*/2,
                                /*wheelbase=*/params.a + params.b,
                                /*rear_length=*/params.b);
}

static vsim::LowSpeedSafety make_mb_safety(const vm::VehicleParameters& params,
                                           const vsim::LowSpeedSafetyConfig& cfg)
{
    return vsim::LowSpeedSafety(cfg,
                                /*longitudinal_index=*/3,
                                /*lateral_index=*/10,
                                /*yaw_rate_index=*/5,
                                /*slip_index=*/std::nullopt,
                                /*wheel_speed_indices=*/{23, 24, 25, 26},
                                /*steering_index=*/2,
                                /*wheelbase=*/params.a + params.b,
                                /*rear_length=*/params.b);
}

static void expect_finite_state(const std::vector<double>& state, const std::string& label)
{
    for (std::size_t i = 0; i < state.size(); ++i) {
        if (!std::isfinite(state[i])) {
            throw std::runtime_error(std::string(label) + " state has non-finite entry at index " +
                                     std::to_string(i));
        }
    }
}

static void test_std_dt_bound()
{
    vio::ConfigManager configs{};
    const auto cfg     = configs.load_low_speed_safety_config(vsim::ModelType::STD);
    const auto profile = cfg.active_profile(cfg.drift_enabled);
    auto       params  = vm::parameters_vehicle1(VELOX_PARAM_ROOT);
    auto       iface   = build_std_interface();
    auto       safety  = make_std_safety(params, cfg);
    const float dt     = configs.load_model_timing(vsim::ModelType::STD).max_dt;
    vsim::VehicleSimulator simulator(std::move(iface), params, dt, std::move(safety));
    simulator.reset(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    expect_finite_state(simulator.state(), "STD initial");

    const std::vector<double> control{0.0, 1.5};
    constexpr int kIterations = 2000;
    for (int i = 0; i < kIterations; ++i) {
        simulator.step(control);
        expect_finite_state(simulator.state(), "STD step " + std::to_string(i));
    }

    expect_finite_state(simulator.state(), "STD final");
    assert(simulator.speed() > profile.release_speed);
    assert(!simulator.safety().engaged());
}

static void test_mb_dt_bound()
{
    vio::ConfigManager configs{};
    const auto cfg     = configs.load_low_speed_safety_config(vsim::ModelType::MB);
    const auto profile = cfg.active_profile(cfg.drift_enabled);
    auto       params  = vm::parameters_vehicle1(VELOX_PARAM_ROOT);
    auto       iface   = build_mb_interface();
    auto       safety  = make_mb_safety(params, cfg);
    const float dt     = configs.load_model_timing(vsim::ModelType::MB).max_dt;
    vsim::VehicleSimulator simulator(std::move(iface), params, dt, std::move(safety));
    simulator.reset(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    expect_finite_state(simulator.state(), "MB initial");

    const std::vector<double> control{0.0, 1.0};
    constexpr int kIterations = 2000;
    for (int i = 0; i < kIterations; ++i) {
        simulator.step(control);
        expect_finite_state(simulator.state(), "MB step " + std::to_string(i));
    }

    expect_finite_state(simulator.state(), "MB final");
    assert(simulator.speed() > profile.release_speed);
    assert(!simulator.safety().engaged());
}

static void test_std_drift_dt_sweep()
{
    vio::ConfigManager configs{};
    const auto timing_info = configs.load_model_timing(vsim::ModelType::STD);
    vsim::ModelTiming    timing(timing_info);

    auto params               = vm::parameters_vehicle1(VELOX_PARAM_ROOT);
    auto iface                = build_std_interface();
    auto safety_cfg           = configs.load_low_speed_safety_config(vsim::ModelType::STD);
    const auto drift_profile  = safety_cfg.active_profile(true);
    auto       safety         = make_std_safety(params, safety_cfg);
    safety.set_drift_enabled(true);

    vsim::VehicleSimulator simulator(std::move(iface), params, timing_info.nominal_dt, std::move(safety));

    const std::vector<double> initial_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const std::vector<double> control{0.0, 1.5};
    const std::vector<float>  requested_steps{timing_info.nominal_dt, timing_info.max_dt, 0.012f, 0.016f};

    auto run_sweep = [&](float requested_dt, const std::string& label) {
        simulator.reset(initial_state);
        timing.reset();

        constexpr int kIterations = 200;
        for (int i = 0; i < kIterations; ++i) {
            const auto schedule = timing.plan_steps(requested_dt);

            double accumulated = 0.0;
            for (double sub_dt : schedule.substeps) {
                assert(sub_dt <= timing_info.max_dt + 1e-12);
                simulator.set_dt(sub_dt);
                simulator.step(control);
                expect_finite_state(simulator.state(), label + " step " + std::to_string(i));
                timing.record_step(sub_dt);
                accumulated += sub_dt;
            }

            assert(std::abs(accumulated - schedule.total_duration()) < 1e-12);
            if (requested_dt > timing_info.max_dt) {
                assert(schedule.used_substeps);
                assert(schedule.substeps.size() > 1);
            }
        }

        expect_finite_state(simulator.state(), label + " final");
        assert(simulator.speed() > drift_profile.release_speed);
        assert(!simulator.safety().engaged());
    };

    for (float requested_dt : requested_steps) {
        run_sweep(requested_dt, "STD drift dt sweep");
    }
}

int main()
{
    test_std_dt_bound();
    test_mb_dt_bound();
    test_std_drift_dt_sweep();
    return 0;
}
