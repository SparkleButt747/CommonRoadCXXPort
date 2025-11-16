#include "controllers/longitudinal/rolling_resistance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace velox::controllers::longitudinal {

RollingResistance::RollingResistance(RollingResistanceConfig config, double gravity)
    : config_(config)
    , gravity_(gravity)
{
    config_.validate();
    if (!std::isfinite(gravity_) || gravity_ <= 0.0) {
        throw std::invalid_argument("gravity must be positive and finite");
    }
}

double RollingResistance::force(double speed, double normal_force) const noexcept
{
    if (std::abs(speed) < 1e-3) {
        return 0.0;
    }
    const double base = config_.c_rr * std::max(0.0, normal_force);
    return -std::copysign(base, speed);
}

} // namespace velox::controllers::longitudinal
