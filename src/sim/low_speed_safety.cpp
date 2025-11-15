#include "sim/low_speed_safety.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vehiclemodels::sim {

void LowSpeedSafetyConfig::validate() const
{
    if (engage_speed < 0.0) {
        throw std::invalid_argument("engage_speed must be non-negative");
    }
    if (release_speed <= 0.0) {
        throw std::invalid_argument("release_speed must be positive");
    }
    if (release_speed < engage_speed) {
        throw std::invalid_argument("release_speed must be >= engage_speed");
    }
    if (yaw_rate_limit <= 0.0) {
        throw std::invalid_argument("yaw_rate_limit must be positive");
    }
    if (slip_angle_limit <= 0.0) {
        throw std::invalid_argument("slip_angle_limit must be positive");
    }
    if (stop_speed_epsilon < 0.0) {
        throw std::invalid_argument("stop_speed_epsilon must be non-negative");
    }
}

LowSpeedSafety::LowSpeedSafety(const LowSpeedSafetyConfig& config,
                               std::optional<int> longitudinal_index,
                               std::optional<int> lateral_index,
                               std::optional<int> yaw_rate_index,
                               std::optional<int> slip_index,
                               std::vector<int> wheel_speed_indices,
                               std::optional<int> steering_index,
                               std::optional<double> wheelbase,
                               std::optional<double> rear_length)
    : config_(config)
    , longitudinal_index_(longitudinal_index)
    , lateral_index_(lateral_index)
    , yaw_rate_index_(yaw_rate_index)
    , slip_index_(slip_index)
    , wheel_speed_indices_(std::move(wheel_speed_indices))
    , steering_index_(steering_index)
    , wheelbase_(wheelbase)
    , rear_length_(rear_length)
{
    config_.validate();
}

void LowSpeedSafety::reset()
{
    engaged_ = false;
}

void LowSpeedSafety::apply(std::vector<double>& state, double speed, bool update_latch)
{
    if (update_latch) {
        if (engaged_) {
            if (speed > config_.release_speed) {
                engaged_ = false;
            }
        } else {
            if (speed < config_.engage_speed) {
                engaged_ = true;
            }
        }
    }

    const auto yaw_target = kinematic_yaw_rate(state, speed);
    const auto slip_target = kinematic_slip(state, speed);
    const auto lateral_target = kinematic_lateral_velocity(state, speed);

    if (longitudinal_index_ && index_in_bounds(*longitudinal_index_, state)) {
        const std::size_t idx = static_cast<std::size_t>(*longitudinal_index_);
        double value = state[idx];
        if (value < 0.0) {
            state[idx] = 0.0;
        } else if (!engaged_ && std::fabs(value) <= config_.stop_speed_epsilon) {
            state[idx] = 0.0;
        }
    }

    if (yaw_rate_index_ && index_in_bounds(*yaw_rate_index_, state)) {
        const std::size_t idx = static_cast<std::size_t>(*yaw_rate_index_);
        if (engaged_) {
            const double limit = config_.yaw_rate_limit;
            const double target = yaw_target.value_or(0.0);
            state[idx] = clamp(target, -limit, limit);
        } else {
            const double limit = config_.yaw_rate_limit;
            state[idx] = clamp(state[idx], -limit, limit);
        }
    }

    if (lateral_index_ && index_in_bounds(*lateral_index_, state)) {
        const std::size_t idx = static_cast<std::size_t>(*lateral_index_);
        double value = state[idx];
        if (engaged_) {
            if (lateral_target.has_value()) {
                state[idx] = *lateral_target;
            } else {
                const double limit = config_.stop_speed_epsilon;
                state[idx] = clamp(value, -limit, limit);
            }
        } else if (std::fabs(value) <= config_.stop_speed_epsilon) {
            state[idx] = 0.0;
        }
    }

    if (slip_index_ && index_in_bounds(*slip_index_, state)) {
        const std::size_t idx = static_cast<std::size_t>(*slip_index_);
        if (engaged_) {
            const double limit = config_.slip_angle_limit;
            const double target = slip_target.value_or(0.0);
            state[idx] = clamp(target, -limit, limit);
        } else {
            const double limit = config_.slip_angle_limit;
            state[idx] = clamp(state[idx], -limit, limit);
        }
    }

    if (!wheel_speed_indices_.empty()) {
        for (int raw_idx : wheel_speed_indices_) {
            if (!index_in_bounds(raw_idx, state)) {
                continue;
            }
            const std::size_t idx = static_cast<std::size_t>(raw_idx);
            double value = state[idx];
            if (value <= 0.0) {
                state[idx] = 0.0;
            } else if (engaged_ && value <= config_.stop_speed_epsilon) {
                state[idx] = 0.0;
            }
        }
    }
}

bool LowSpeedSafety::index_in_bounds(int index, const std::vector<double>& state)
{
    return index >= 0 && static_cast<std::size_t>(index) < state.size();
}

double LowSpeedSafety::clamp(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(max_value, value));
}

std::optional<double> LowSpeedSafety::kinematic_beta(const std::vector<double>& state) const
{
    if (!steering_index_.has_value() || !wheelbase_.has_value() || !rear_length_.has_value()) {
        return std::nullopt;
    }
    if (!index_in_bounds(*steering_index_, state)) {
        return std::nullopt;
    }
    if (*wheelbase_ <= 0.0) {
        return std::nullopt;
    }
    const double delta = state[static_cast<std::size_t>(*steering_index_)];
    const double ratio = *rear_length_ / *wheelbase_;
    return std::atan(std::tan(delta) * ratio);
}

std::optional<double> LowSpeedSafety::kinematic_yaw_rate(const std::vector<double>& state, double speed) const
{
    const auto beta = kinematic_beta(state);
    if (!beta.has_value() || !steering_index_.has_value() || !wheelbase_.has_value()) {
        return std::nullopt;
    }
    if (!index_in_bounds(*steering_index_, state)) {
        return std::nullopt;
    }
    if (std::fabs(speed) <= 1e-9) {
        return 0.0;
    }
    const double delta = state[static_cast<std::size_t>(*steering_index_)];
    if (*wheelbase_ <= 0.0) {
        return std::nullopt;
    }
    return speed * std::cos(*beta) * std::tan(delta) / *wheelbase_;
}

std::optional<double> LowSpeedSafety::kinematic_slip(const std::vector<double>& state, double)
    const
{
    (void)state;
    return kinematic_beta(state);
}

std::optional<double> LowSpeedSafety::kinematic_lateral_velocity(const std::vector<double>& state, double speed) const
{
    const auto beta = kinematic_beta(state);
    if (!beta.has_value()) {
        return std::nullopt;
    }
    return speed * std::sin(*beta);
}

} // namespace vehiclemodels::sim
