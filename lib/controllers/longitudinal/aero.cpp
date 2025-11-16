#include "controllers/longitudinal/aero.hpp"

#include <cmath>

namespace velox::controllers::longitudinal {

AeroModel::AeroModel(AeroConfig config)
    : config_(config)
{
    config_.validate();
}

double AeroModel::drag_force(double speed) const noexcept
{
    if (speed == 0.0) {
        return 0.0;
    }
    const double coeff     = config_.drag_coefficient;
    const double magnitude = coeff * speed * speed;
    return -std::copysign(magnitude, speed);
}

double AeroModel::downforce(double speed) const noexcept
{
    if (config_.downforce_coefficient == 0.0 || speed == 0.0) {
        return 0.0;
    }
    const double coeff = std::abs(config_.downforce_coefficient);
    return coeff * speed * speed;
}

} // namespace velox::controllers::longitudinal
