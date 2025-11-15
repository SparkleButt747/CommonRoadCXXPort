#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace vehiclemodels::sim::longitudinal {

struct PowertrainConfig {
    double max_drive_torque      = 0.0;
    double max_regen_torque      = 0.0;
    double max_power             = 0.0;
    double drive_efficiency      = 1.0;
    double regen_efficiency      = 1.0;
    double min_soc               = 0.0;
    double max_soc               = 1.0;
    double initial_soc           = 0.0;
    double battery_capacity_kwh  = 0.0;

    void validate() const
    {
        auto finite_and_non_negative = [](double value, const char* name) {
            if (!std::isfinite(value) || value < 0.0) {
                throw std::invalid_argument(std::string{name} + " must be non-negative and finite");
            }
        };

        finite_and_non_negative(max_drive_torque, "powertrain.max_drive_torque");
        finite_and_non_negative(max_regen_torque, "powertrain.max_regen_torque");
        if (!std::isfinite(max_power) || max_power < 0.0) {
            throw std::invalid_argument("powertrain.max_power must be non-negative and finite");
        }
        finite_and_non_negative(drive_efficiency, "powertrain.drive_efficiency");
        finite_and_non_negative(regen_efficiency, "powertrain.regen_efficiency");
        if (drive_efficiency <= 0.0 || drive_efficiency > 1.0) {
            throw std::invalid_argument("powertrain.drive_efficiency must be in (0, 1]");
        }
        if (regen_efficiency < 0.0 || regen_efficiency > 1.0) {
            throw std::invalid_argument("powertrain.regen_efficiency must be in [0, 1]");
        }
        if (!std::isfinite(min_soc) || !std::isfinite(max_soc) || !std::isfinite(initial_soc)) {
            throw std::invalid_argument("powertrain SOC bounds must be finite");
        }
        if (min_soc < 0.0 || max_soc > 1.0) {
            throw std::invalid_argument("powertrain SOC bounds must lie within [0, 1]");
        }
        if (!(min_soc <= initial_soc && initial_soc <= max_soc)) {
            throw std::invalid_argument("0 <= min_soc <= initial_soc <= max_soc <= 1 must hold");
        }
        if (!std::isfinite(battery_capacity_kwh) || battery_capacity_kwh <= 0.0) {
            throw std::invalid_argument("powertrain.battery_capacity_kwh must be positive and finite");
        }
    }
};

struct PowertrainOutput {
    double total_torque = 0.0;
    double drive_torque = 0.0;
    double regen_torque = 0.0;
};

class Powertrain {
public:
    Powertrain(PowertrainConfig config, double wheel_radius);

    [[nodiscard]] double available_drive_torque(double speed) const noexcept;

    [[nodiscard]] double available_regen_torque(double speed) const noexcept;

    [[nodiscard]] PowertrainOutput step(double throttle,
                                        double regen_torque_request,
                                        double speed,
                                        double dt);

    void reset() noexcept;

    [[nodiscard]] double soc() const noexcept { return soc_; }
    [[nodiscard]] const PowertrainConfig& config() const noexcept { return config_; }

private:
    [[nodiscard]] double torque_power_limited(double speed) const noexcept;

    PowertrainConfig config_;
    double wheel_radius_;
    double capacity_joules_;
    double soc_;
};

} // namespace vehiclemodels::sim::longitudinal
