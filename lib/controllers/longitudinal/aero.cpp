#include "controllers/longitudinal/aero.hpp"

#include <cmath>
#include <stdexcept>

namespace velox::controllers::longitudinal {

void AeroConfig::validate() const
{
    if (!std::isfinite(drag_coefficient)) {
        throw std::invalid_argument("aero.drag_coefficient must be finite");
    }
    if (!std::isfinite(downforce_coefficient)) {
        throw std::invalid_argument("aero.downforce_coefficient must be finite");
    }
    if (drag_coefficient < 0.0) {
        throw std::invalid_argument("aero.drag_coefficient cannot be negative");
    }
}

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
