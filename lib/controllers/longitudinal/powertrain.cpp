#include "controllers/longitudinal/powertrain.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace velox::controllers::longitudinal {

Powertrain::Powertrain(PowertrainConfig config, double wheel_radius)
    : config_(config)
    , wheel_radius_(wheel_radius)
    , capacity_joules_(config.battery_capacity_kwh * 3.6e6)
    , soc_(config.initial_soc)
{
    config_.validate();
    if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0) {
        throw std::invalid_argument("wheel_radius must be positive and finite");
    }
    if (!std::isfinite(capacity_joules_) || capacity_joules_ <= 0.0) {
        throw std::invalid_argument("battery capacity must be positive");
    }
}

double Powertrain::available_drive_torque(double speed) const noexcept
{
    if (soc_ <= config_.min_soc) {
        return 0.0;
    }
    const double limit = torque_power_limited(speed);
    return std::clamp(limit, 0.0, config_.max_drive_torque);
}

double Powertrain::available_regen_torque(double speed) const noexcept
{
    if (soc_ >= config_.max_soc) {
        return 0.0;
    }
    if (std::abs(speed) < 1e-3) {
        return 0.0;
    }
    const double limit = torque_power_limited(speed);
    return std::clamp(limit, 0.0, config_.max_regen_torque);
}

PowertrainOutput Powertrain::step(double throttle,
                                  double regen_torque_request,
                                  double speed,
                                  double dt)
{
    throttle             = std::clamp(throttle, 0.0, 1.0);
    regen_torque_request = std::max(0.0, regen_torque_request);

    const double drive_limit  = available_drive_torque(speed);
    const double drive_torque = std::min(throttle * config_.max_drive_torque, drive_limit);
    const double regen_limit  = available_regen_torque(speed);
    const double regen_torque = std::min(regen_torque_request, regen_limit);

    const double wheel_speed = speed / wheel_radius_;
    double mechanical_drive_power = drive_torque * wheel_speed;
    double mechanical_regen_power = -regen_torque * wheel_speed;

    double battery_power = 0.0;
    if (mechanical_drive_power > 0.0) {
        battery_power += mechanical_drive_power / std::max(config_.drive_efficiency, 1e-6);
    } else {
        mechanical_drive_power = 0.0;
    }
    if (mechanical_regen_power < 0.0) {
        battery_power += mechanical_regen_power * config_.regen_efficiency;
    } else {
        mechanical_regen_power = 0.0;
    }

    const double soc_delta = (dt > 0.0) ? -battery_power * dt / capacity_joules_ : 0.0;
    soc_                   = std::clamp(soc_ + soc_delta, config_.min_soc, config_.max_soc);

    const double total_torque = drive_torque - regen_torque;
    return PowertrainOutput{total_torque, drive_torque, regen_torque};
}

void Powertrain::reset() noexcept
{
    soc_ = config_.initial_soc;
}

double Powertrain::torque_power_limited(double speed) const noexcept
{
    if (config_.max_power <= 0.0) {
        return config_.max_drive_torque;
    }
    const double wheel_speed = std::abs(speed) / wheel_radius_;
    if (wheel_speed < 1e-6) {
        return config_.max_drive_torque;
    }
    return config_.max_power / wheel_speed;
}

} // namespace velox::controllers::longitudinal
