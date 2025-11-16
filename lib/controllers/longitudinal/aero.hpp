#pragma once

#include <cmath>
#include <stdexcept>

namespace velox::controllers::longitudinal {

struct AeroConfig {
    double drag_coefficient           = 0.0;
    double downforce_coefficient      = 0.0;

    void validate() const
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
};

class AeroModel {
public:
    explicit AeroModel(AeroConfig config);

    [[nodiscard]] double drag_force(double speed) const noexcept;

    [[nodiscard]] double downforce(double speed) const noexcept;

    [[nodiscard]] const AeroConfig& config() const noexcept { return config_; }

private:
    AeroConfig config_;
};

} // namespace velox::controllers::longitudinal
