#include "utils/steering_controller.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#ifndef COMMONROAD_PARAM_ROOT
#    error "COMMONROAD_PARAM_ROOT must be defined"
#endif

namespace vehiclemodels {
namespace utils {

namespace {
constexpr double EPS = 1e-9;

double clamp01(double value)
{
    if (value < 0.0) return 0.0;
    if (value > 1.0) return 1.0;
    return value;
}

SteeringConfig parse_config(const YAML::Node& root)
{
    SteeringConfig cfg{};

    auto wheel = root["wheel"];
    if (!wheel) {
        throw std::runtime_error("steering config missing 'wheel' section");
    }
    cfg.wheel.max_angle           = wheel["max_angle"].as<double>();
    cfg.wheel.max_rate            = wheel["max_rate"].as<double>();
    cfg.wheel.time_constant       = wheel["time_constant"].as<double>(0.0);

    auto centering = wheel["centering"];
    if (!centering) {
        throw std::runtime_error("steering config missing 'wheel.centering' section");
    }
    cfg.wheel.centering_stiffness = centering["stiffness"].as<double>();
    cfg.wheel.centering_deadband  = centering["deadband"].as<double>();

    auto actuator = root["actuator"];
    if (!actuator) {
        throw std::runtime_error("steering config missing 'actuator' section");
    }
    cfg.actuator.time_constant = actuator["time_constant"].as<double>();

    return cfg;
}

} // namespace

SteeringConfig SteeringConfig::load_from_file(const std::string& path)
{
    YAML::Node root = YAML::LoadFile(path);
    return parse_config(root);
}

SteeringConfig SteeringConfig::load_default()
{
    namespace fs = std::filesystem;
    fs::path root = fs::path(COMMONROAD_PARAM_ROOT).parent_path();
    fs::path path = root / "config" / "steering.yaml";
    if (!fs::exists(path)) {
        throw std::runtime_error("steering config not found at: " + path.string());
    }
    return load_from_file(path.string());
}

SteeringWheel::SteeringWheel(const SteeringConfig::Wheel& cfg,
                             const SteeringParameters& limits)
    : cfg_(cfg)
    , limits_(limits)
{
    reset(0.0);
}

double SteeringWheel::max_left() const
{
    double limit = limits_.max;
    if (cfg_.max_angle > EPS) {
        limit = std::min(limit, cfg_.max_angle);
    }
    return limit;
}

double SteeringWheel::max_right() const
{
    double limit = limits_.min;
    if (cfg_.max_angle > EPS) {
        limit = std::max(limit, -cfg_.max_angle);
    }
    return limit;
}

void SteeringWheel::reset(double angle)
{
    target_angle_        = angle;
    this->angle_         = angle;
    last_output_.angle   = angle;
    last_output_.rate    = 0.0;
    last_output_.target_angle = angle;
}

SteeringWheel::Output SteeringWheel::update(double input, double dt)
{
    if (dt <= 0.0) {
        return last_output_;
    }

    double clamped_input = std::clamp(input, -1.0, 1.0);
    double rate_command  = clamped_input * cfg_.max_rate;

    if (std::abs(clamped_input) < EPS) {
        const double deviation = target_angle_;
        if (std::abs(deviation) > cfg_.centering_deadband) {
            const double error       = std::abs(deviation) - cfg_.centering_deadband;
            double       return_rate = cfg_.centering_stiffness * error;
            return_rate             = std::min(return_rate, cfg_.max_rate);
            rate_command            = -std::copysign(return_rate, deviation);
        } else {
            rate_command = 0.0;
        }
    }

    target_angle_ += rate_command * dt;
    target_angle_ = std::clamp(target_angle_, max_right(), max_left());

    const double prev_angle = angle_;
    if (cfg_.time_constant <= 0.0) {
        angle_ = target_angle_;
    } else {
        const double alpha = clamp01(dt / std::max(cfg_.time_constant, dt));
        angle_ += (target_angle_ - angle_) * alpha;
    }

    const double rate = (angle_ - prev_angle) / dt;

    last_output_.target_angle = target_angle_;
    last_output_.angle        = angle_;
    last_output_.rate         = rate;
    return last_output_;
}

FinalSteerController::FinalSteerController(const SteeringConfig::Actuator& cfg,
                                           const SteeringParameters& limits)
    : cfg_(cfg)
    , limits_(limits)
{
    reset(0.0);
}

void FinalSteerController::reset(double current_angle)
{
    filtered_target_         = current_angle;
    last_output_.filtered_target = current_angle;
    last_output_.angle        = current_angle;
    last_output_.rate         = 0.0;
}

FinalSteerController::Output FinalSteerController::update(double desired_angle,
                                                          double current_angle,
                                                          double dt)
{
    if (dt <= 0.0) {
        return last_output_;
    }

    const double target_limited = std::clamp(desired_angle, limits_.min, limits_.max);

    if (cfg_.time_constant <= 0.0) {
        filtered_target_ = target_limited;
    } else {
        const double alpha = clamp01(dt / std::max(cfg_.time_constant, dt));
        filtered_target_ += (target_limited - filtered_target_) * alpha;
    }

    double desired_rate;
    if (cfg_.time_constant <= 0.0) {
        desired_rate = (target_limited - current_angle) / dt;
    } else {
        desired_rate = (filtered_target_ - current_angle) / cfg_.time_constant;
    }

    double rate_min = limits_.v_min;
    double rate_max = limits_.v_max;
    if (rate_min >= rate_max) {
        const double fallback = std::max(std::abs(rate_min), std::abs(rate_max));
        rate_min              = -fallback;
        rate_max              = fallback;
        if (fallback < EPS) {
            rate_min = -1.0;
            rate_max = 1.0;
        }
    }

    double rate = std::clamp(desired_rate, rate_min, rate_max);
    double next_angle = current_angle + rate * dt;

    if (next_angle > limits_.max) {
        next_angle = limits_.max;
        rate       = (next_angle - current_angle) / dt;
    } else if (next_angle < limits_.min) {
        next_angle = limits_.min;
        rate       = (next_angle - current_angle) / dt;
    }

    last_output_.filtered_target = filtered_target_;
    last_output_.angle           = next_angle;
    last_output_.rate            = rate;
    return last_output_;
}

} // namespace utils
} // namespace vehiclemodels
