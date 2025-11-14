#pragma once

#include <algorithm>
#include <string>

#include "utils/steering_parameters.hpp"

namespace vehiclemodels {
namespace utils {

struct SteeringConfig {
    struct Wheel {
        double max_angle{0.0};
        double max_rate{0.0};
        double time_constant{0.0};
        double centering_stiffness{0.0};
        double centering_deadband{0.0};
    };

    struct Actuator {
        double time_constant{0.0};
    };

    Wheel    wheel{};
    Actuator actuator{};

    static SteeringConfig load_from_file(const std::string& path);
    static SteeringConfig load_default();
};

class SteeringWheel {
public:
    SteeringWheel() = default;
    SteeringWheel(const SteeringConfig::Wheel& cfg,
                  const SteeringParameters& limits);

    struct Output {
        double target_angle{0.0};
        double angle{0.0};
        double rate{0.0};
    };

    Output update(double input, double dt);
    void   reset(double angle = 0.0);

    const Output& last_output() const { return last_output_; }

private:
    SteeringConfig::Wheel cfg_{};
    SteeringParameters    limits_{};
    double                target_angle_{0.0};
    double                angle_{0.0};
    Output                last_output_{};

    [[nodiscard]] double max_left() const;
    [[nodiscard]] double max_right() const;
};

class FinalSteerController {
public:
    FinalSteerController() = default;
    FinalSteerController(const SteeringConfig::Actuator& cfg,
                         const SteeringParameters& limits);

    struct Output {
        double filtered_target{0.0};
        double angle{0.0};
        double rate{0.0};
    };

    Output update(double desired_angle, double current_angle, double dt);
    void   reset(double current_angle = 0.0);

    const Output& last_output() const { return last_output_; }

private:
    SteeringConfig::Actuator cfg_{};
    SteeringParameters       limits_{};
    double                   filtered_target_{0.0};
    Output                   last_output_{};
};

} // namespace utils
} // namespace vehiclemodels
