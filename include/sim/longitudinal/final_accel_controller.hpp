#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "sim/longitudinal/aero.hpp"
#include "sim/longitudinal/brake.hpp"
#include "sim/longitudinal/powertrain.hpp"
#include "sim/longitudinal/rolling_resistance.hpp"

namespace vehiclemodels::sim::longitudinal {

struct FinalAccelControllerConfig {
    double tau_throttle       = 0.1;
    double tau_brake          = 0.1;
    double accel_min          = -8.0;
    double accel_max          = 4.0;
    double stop_speed_epsilon = 0.05;

    void validate() const
    {
        if (!std::isfinite(tau_throttle) || tau_throttle <= 0.0) {
            throw std::invalid_argument("final_accel_controller.tau_throttle must be positive");
        }
        if (!std::isfinite(tau_brake) || tau_brake <= 0.0) {
            throw std::invalid_argument("final_accel_controller.tau_brake must be positive");
        }
        if (!std::isfinite(accel_min) || !std::isfinite(accel_max)) {
            throw std::invalid_argument("final_accel_controller.accel bounds must be finite");
        }
        if (accel_max < accel_min) {
            throw std::invalid_argument("final_accel_controller.accel_max must be >= accel_min");
        }
        if (!std::isfinite(stop_speed_epsilon) || stop_speed_epsilon < 0.0) {
            throw std::invalid_argument("final_accel_controller.stop_speed_epsilon must be non-negative");
        }
    }
};

struct DriverIntent {
    double throttle = 0.0;
    double brake    = 0.0;
};

struct ControllerOutput {
    double acceleration    = 0.0;
    double throttle        = 0.0;
    double brake           = 0.0;
    double drive_force     = 0.0;
    double brake_force     = 0.0;
    double regen_force     = 0.0;
    double hydraulic_force = 0.0;
    double drag_force      = 0.0;
    double rolling_force   = 0.0;
};

class FinalAccelController {
public:
    FinalAccelController(double vehicle_mass,
                         double wheel_radius,
                         PowertrainConfig powertrain_cfg,
                         AeroConfig aero_cfg,
                         RollingResistanceConfig rolling_cfg,
                         BrakeConfig brake_cfg,
                         FinalAccelControllerConfig controller_cfg);

    ControllerOutput step(const DriverIntent& intent, double speed, double dt);

    void reset();

    [[nodiscard]] double throttle() const noexcept { return throttle_; }
    [[nodiscard]] double brake() const noexcept { return brake_; }
    [[nodiscard]] const Powertrain& powertrain() const noexcept { return powertrain_; }
    [[nodiscard]] const FinalAccelControllerConfig& config() const noexcept { return cfg_; }

private:
    void apply_actuator_dynamics(const DriverIntent& intent, double dt);

    double mass_;
    double wheel_radius_;
    Powertrain powertrain_;
    AeroModel aero_;
    RollingResistance rolling_;
    BrakeController brakes_;
    FinalAccelControllerConfig cfg_;

    double throttle_ = 0.0;
    double brake_    = 0.0;
};

} // namespace vehiclemodels::sim::longitudinal
