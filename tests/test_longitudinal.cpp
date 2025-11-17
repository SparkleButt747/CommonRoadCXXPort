#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "io/config_manager.hpp"
#include "controllers/longitudinal/final_accel_controller.hpp"
#include "vehicle/parameters_vehicle2.hpp"

namespace vm  = velox::models;
namespace vml = velox::controllers::longitudinal;
namespace vio = velox::io;

void test_soc_bounds_validation()
{
    vml::PowertrainConfig cfg{};
    cfg.max_drive_torque     = 320.0;
    cfg.max_regen_torque     = 120.0;
    cfg.max_power            = 130000.0;
    cfg.drive_efficiency     = 0.92;
    cfg.regen_efficiency     = 0.65;
    cfg.min_soc              = 0.6;
    cfg.max_soc              = 0.9;
    cfg.initial_soc          = 0.5;
    cfg.battery_capacity_kwh = 18.0;

    bool threw = false;
    try {
        cfg.validate();
    } catch (const std::exception&) {
        threw = true;
    }
    assert(threw && "PowertrainConfig should reject invalid SOC ordering");
}

void test_regen_fade_out()
{
    vio::ConfigManager configs{};
    const auto brake_cfg = configs.load_brake_config();
    vml::BrakeController brakes(brake_cfg);

    const double speed_above = brake_cfg.min_regen_speed + 0.5;
    const double speed_below = std::max(0.0, brake_cfg.min_regen_speed - 0.5);
    const double available   = brake_cfg.max_force;

    const auto above = brakes.blend(1.0, speed_above, available);
    const auto below = brakes.blend(1.0, speed_below, available);

    assert(above.total_force == above.hydraulic_force + above.regen_force);
    assert(below.total_force == below.hydraulic_force + below.regen_force);
    assert(std::abs(above.total_force - below.total_force) < 1e-6);
    assert(above.regen_force > 0.0);
    assert(below.regen_force >= 0.0);
    assert(above.regen_force > below.regen_force);
    assert(below.hydraulic_force > above.hydraulic_force);
}

void test_stop_and_go_continuity()
{
    vio::ConfigManager configs{};
    const auto params      = vm::parameters_vehicle2();
    const auto power_cfg   = configs.load_powertrain_config();
    const auto aero_cfg    = configs.load_aero_config();
    const auto rolling_cfg = configs.load_rolling_resistance_config();
    const auto brake_cfg   = configs.load_brake_config();
    const auto ctrl_cfg    = configs.load_final_accel_controller_config();

    vml::FinalAccelController controller(params.m,
                                         params.R_w,
                                         power_cfg,
                                         aero_cfg,
                                         rolling_cfg,
                                         brake_cfg,
                                         ctrl_cfg);

    const double dt = 0.05;
    double speed    = 5.0;

    double accel_at_stop = 0.0;
    double stop_speed     = std::numeric_limits<double>::infinity();
    bool   reached_stop   = false;
    for (int i = 0; i < 400; ++i) {
        const auto output = controller.step(vml::DriverIntent{0.0, 1.0}, speed, dt);
        speed              = std::max(0.0, speed + output.acceleration * dt);
        if (speed <= ctrl_cfg.stop_speed_epsilon + 1e-6) {
            stop_speed   = speed;
            reached_stop = true;
            const auto settle = controller.step(vml::DriverIntent{0.0, 1.0}, speed, dt);
            accel_at_stop = settle.acceleration;
            break;
        }
    }
    assert(reached_stop);
    assert(stop_speed <= ctrl_cfg.stop_speed_epsilon + 1e-6);
    assert(accel_at_stop >= -1e-6);

    bool resumed_motion = false;
    for (int i = 0; i < 200; ++i) {
        const auto output = controller.step(vml::DriverIntent{1.0, 0.0}, speed, dt);
        speed              = std::max(0.0, speed + output.acceleration * dt);
        if (output.acceleration > 0.0) {
            resumed_motion = true;
            break;
        }
    }
    assert(resumed_motion && "Controller should resume acceleration after stop");
}

int main()
{
    try {
        test_soc_bounds_validation();
        test_regen_fade_out();
        test_stop_and_go_continuity();
    } catch (const std::exception& ex) {
        std::cerr << "Test raised exception: " << ex.what() << '\n';
        return 1;
    }
    std::cout << "All longitudinal tests passed\n";
    return 0;
}
