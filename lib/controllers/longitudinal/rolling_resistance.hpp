#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace velox::controllers::longitudinal {

struct RollingResistanceConfig {
    double c_rr = 0.0;

    void validate() const
    {
        if (!std::isfinite(c_rr)) {
            throw std::invalid_argument("rolling_resistance.c_rr must be finite");
        }
        if (c_rr < 0.0) {
            throw std::invalid_argument("rolling_resistance.c_rr cannot be negative");
        }
    }
};

class RollingResistance {
public:
    RollingResistance(RollingResistanceConfig config, double gravity = 9.81);

    [[nodiscard]] double force(double speed, double normal_force) const noexcept;

    [[nodiscard]] const RollingResistanceConfig& config() const noexcept { return config_; }

private:
    RollingResistanceConfig config_;
    double gravity_;
};

} // namespace velox::controllers::longitudinal
