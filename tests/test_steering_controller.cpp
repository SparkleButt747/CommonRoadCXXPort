#include <cmath>
#include <iostream>

#include "utils/steering_controller.hpp"

using vehiclemodels::utils::FinalSteerController;
using vehiclemodels::utils::SteeringConfig;
using vehiclemodels::utils::SteeringParameters;
using vehiclemodels::utils::SteeringWheel;

namespace {
SteeringConfig::Wheel make_wheel_config()
{
    SteeringConfig::Wheel cfg;
    cfg.max_angle           = 0.6;
    cfg.max_rate            = 3.5;
    cfg.nudge_angle         = 0.06;
    cfg.centering_stiffness = 5.0;
    cfg.centering_deadband  = 0.01;
    return cfg;
}

SteeringParameters make_limits()
{
    SteeringParameters limits;
    limits.min   = -0.6;
    limits.max   = 0.6;
    limits.v_min = -0.7;
    limits.v_max = 0.7;
    return limits;
}

SteeringConfig::Final make_final_config()
{
    SteeringConfig::Final cfg;
    cfg.min_angle               = -0.6;
    cfg.max_angle               = 0.6;
    cfg.max_rate                = 2.5;
    cfg.actuator_time_constant  = 0.18;
    cfg.smoothing_time_constant = 0.12;
    return cfg;
}

bool nearly_equal(double a, double b, double tol = 1e-3)
{
    return std::abs(a - b) <= tol;
}

} // namespace

int main()
{
    const double dt = 0.05;

    // Test automatic centering of steering wheel
    {
        SteeringWheel wheel(make_wheel_config(), make_limits());
        for (int i = 0; i < 4; ++i) {
            wheel.update(1.0, dt); // multiple nudges left
        }
        for (int i = 0; i < 80; ++i) {
            wheel.update(0.0, dt);
        }
        const auto& out = wheel.last_output();
        if (std::abs(out.angle) > 0.05) {
            std::cerr << "Centering test failed: angle=" << out.angle << '\n';
            return 1;
        }
        if (std::abs(out.rate) > 0.4) {
            std::cerr << "Centering test failed: rate=" << out.rate << '\n';
            return 1;
        }
    }

    // Test saturation at maximum steering
    {
        SteeringWheel wheel(make_wheel_config(), make_limits());
        for (int i = 0; i < 80; ++i) {
            wheel.update(1.0, dt);
        }
        const auto& out = wheel.last_output();
        const double max_expected = std::min(make_wheel_config().max_angle, make_limits().max);
        if (out.angle > make_limits().max + 1e-6) {
            std::cerr << "Saturation test failed: angle=" << out.angle << '\n';
            return 1;
        }
        if (!nearly_equal(out.angle, max_expected, 0.05)) {
            std::cerr << "Saturation test did not reach expected max angle\n";
            return 1;
        }
    }

    // Test smooth transition of final controller
    {
        FinalSteerController controller(make_final_config(), make_limits());
        double current_angle = 0.0;
        const double desired = 0.5;
        auto first = controller.update(desired, current_angle, dt);
        if (first.angle >= desired - 1e-4) {
            std::cerr << "Smoothing test failed: controller jumped to desired angle\n";
            return 1;
        }
        if (std::abs(first.rate) > make_limits().v_max + 1e-6) {
            std::cerr << "Smoothing test failed: rate limit exceeded\n";
            return 1;
        }

        double prev_rate = first.rate;
        current_angle    = first.angle;
        for (int i = 0; i < 30; ++i) {
            auto out = controller.update(desired, current_angle, dt);
            if (std::abs(out.rate) > make_limits().v_max + 1e-6) {
                std::cerr << "Smoothing test failed: rate limit exceeded\n";
                return 1;
            }
            if (std::abs(out.rate - prev_rate) > 0.5) {
                std::cerr << "Smoothing test failed: rate change too abrupt\n";
                return 1;
            }
            current_angle = out.angle;
            prev_rate     = out.rate;
        }

        if (!nearly_equal(current_angle, desired, 0.05)) {
            std::cerr << "Smoothing test failed: angle did not approach desired\n";
            return 1;
        }
    }

    std::cout << "All steering controller tests passed\n";
    return 0;
}
