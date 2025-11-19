#include "simulation/low_speed_safety.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include "common/errors.hpp"
namespace velox::simulation {

void LowSpeedSafetyProfile::validate(const char* name) const
{
    const auto error = [name](std::string_view field, std::string_view msg) {
        std::ostringstream oss;
        oss << name << "." << field << " " << msg;
        return ::velox::errors::ConfigError(VELOX_LOC(oss.str()));
    };

    if (engage_speed < 0.0) {
        throw error("engage_speed", "must be non-negative");
    }
    if (release_speed <= 0.0) {
        throw error("release_speed", "must be positive");
    }
    if (release_speed < engage_speed) {
        throw error("release_speed", "must be >= engage_speed");
    }
    if (yaw_rate_limit <= 0.0) {
        throw error("yaw_rate_limit", "must be positive");
    }
    if (slip_angle_limit <= 0.0) {
        throw error("slip_angle_limit", "must be positive");
    }
}

void LowSpeedSafetyConfig::validate() const
{
    normal.validate("normal");
    drift.validate("drift");
    if (stop_speed_epsilon < 0.0) {
        throw ::velox::errors::ConfigError(VELOX_LOC("stop_speed_epsilon must be non-negative"));
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
    drift_enabled_ = config_.drift_enabled;
}

void LowSpeedSafety::reset()
{
    engaged_ = false;
}

void LowSpeedSafety::apply(std::vector<double>& state, double speed, bool update_latch)
{
    const auto& profile = config_.active_profile(drift_enabled_);

    if (update_latch) {
        if (engaged_) {
            if (speed > profile.release_speed) {
                engaged_ = false;
            }
        } else {
            if (speed < profile.engage_speed) {
                engaged_ = true;
            }
        }
    }

    const bool latch_active = engaged_;
    const bool drift_mode   = drift_enabled_;
    const bool near_stop    = std::abs(speed) <= config_.stop_speed_epsilon;
    const double transition_blend = pre_latch_blend(speed, profile);

    auto yaw_target       = kinematic_yaw_rate(state, speed);
    auto slip_target      = kinematic_slip(state, speed);
    auto lateral_target   = kinematic_lateral_velocity(state, speed);
    auto velocity_heading = velocity_slip(state);

    if (latch_active) {
        const double beta_ref     = velocity_heading.value_or(0.0);
        const double slip_command = (velocity_heading.has_value() && !near_stop) ? beta_ref : 0.0;

        yaw_target     = 0.0;
        slip_target    = slip_command;
        if (velocity_heading.has_value() && !near_stop) {
            lateral_target = speed * std::sin(slip_command);
        } else {
            lateral_target = 0.0;
        }
    }

    if (yaw_rate_index_ && index_in_bounds(*yaw_rate_index_, state)) {
        const std::size_t idx = static_cast<std::size_t>(*yaw_rate_index_);
        const bool         allow_unclamped = drift_mode && !latch_active && transition_blend <= 0.0;
        if (latch_active) {
            const double limit  = profile.yaw_rate_limit;
            const double target = yaw_target.value_or(0.0);
            state[idx]          = clamp(target, -limit, limit);
        } else if (!allow_unclamped) {
            double limit = scaled_limit(profile.yaw_rate_limit, speed, profile);
            double value = clamp(state[idx], -limit, limit);
            if (transition_blend > 0.0 && yaw_target.has_value()) {
                const double target = clamp(*yaw_target, -limit, limit);
                value = (1.0 - transition_blend) * value + transition_blend * target;
            }
            state[idx] = value;
        }
    }

    if (lateral_index_ && index_in_bounds(*lateral_index_, state)) {
        const std::size_t idx = static_cast<std::size_t>(*lateral_index_);
        double value = state[idx];
        if (latch_active) {
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
        const bool         allow_unclamped = drift_mode && !latch_active && transition_blend <= 0.0;
        if (latch_active) {
            const double limit  = profile.slip_angle_limit;
            const double target = slip_target.value_or(0.0);
            state[idx]          = clamp(target, -limit, limit);
        } else if (!allow_unclamped) {
            double limit = scaled_limit(profile.slip_angle_limit, speed, profile);
            double value = clamp(state[idx], -limit, limit);
            double target = 0.0;
            if (velocity_heading.has_value() && !near_stop) {
                target = clamp(*velocity_heading, -limit, limit);
            }
            if (transition_blend > 0.0) {
                value = (1.0 - transition_blend) * value + transition_blend * target;
            }
            state[idx] = value;
        }
    }

    if (!wheel_speed_indices_.empty()) {
        const bool wheel_stage_latch = latch_active || speed < profile.engage_speed || transition_blend > 0.0;

        for (int raw_idx : wheel_speed_indices_) {
            if (!index_in_bounds(raw_idx, state)) {
                continue;
            }
            const std::size_t idx = static_cast<std::size_t>(raw_idx);
            double value = state[idx];
            if (value <= 0.0) {
                state[idx] = 0.0;
            } else if (wheel_stage_latch && value <= config_.stop_speed_epsilon) {
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

std::optional<double> LowSpeedSafety::velocity_slip(const std::vector<double>& state) const
{
    if (!longitudinal_index_.has_value() || !lateral_index_.has_value()) {
        if (slip_index_.has_value() && index_in_bounds(*slip_index_, state)) {
            return state[static_cast<std::size_t>(*slip_index_)];
        }
        return std::nullopt;
    }
    if (!index_in_bounds(*longitudinal_index_, state) || !index_in_bounds(*lateral_index_, state)) {
        return std::nullopt;
    }

    const double longitudinal = state[static_cast<std::size_t>(*longitudinal_index_)];
    const double lateral      = state[static_cast<std::size_t>(*lateral_index_)];
    if (std::abs(longitudinal) <= 1e-9 && std::abs(lateral) <= 1e-9) {
        return 0.0;
    }

    return std::atan2(lateral, longitudinal);
}

double LowSpeedSafety::pre_latch_blend(double speed, const LowSpeedSafetyProfile& profile) const
{
    const double band   = std::max(profile.release_speed - profile.engage_speed, 0.0);
    const double upper  = profile.release_speed + band;
    const double lower  = profile.release_speed;
    if (upper <= lower || speed >= upper) {
        return 0.0;
    }
    if (speed <= lower) {
        return 1.0;
    }
    const double ratio = (upper - speed) / (upper - lower);
    return std::clamp(ratio, 0.0, 1.0);
}

double LowSpeedSafety::scaled_limit(double limit, double speed, const LowSpeedSafetyProfile& profile) const
{
    if (speed >= profile.release_speed || profile.release_speed <= 0.0) {
        return limit;
    }
    const double ratio = std::clamp(speed / std::max(profile.release_speed, 1e-6), 0.0, 1.0);
    const double min_limit = std::max(config_.stop_speed_epsilon, 1e-6);
    return std::clamp(limit * ratio, min_limit, limit);
}

} // namespace velox::simulation
