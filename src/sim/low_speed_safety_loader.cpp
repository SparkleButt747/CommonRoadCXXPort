#include "sim/low_speed_safety_loader.hpp"

#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#ifndef COMMONROAD_PARAM_ROOT
#    error "COMMONROAD_PARAM_ROOT must be defined"
#endif

namespace vehiclemodels::sim {

namespace {
namespace fs = std::filesystem;

LowSpeedSafetyConfig parse_low_speed_config(const YAML::Node& node)
{
    LowSpeedSafetyConfig cfg{};
    cfg.engage_speed       = node["engage_speed"].as<double>();
    cfg.release_speed      = node["release_speed"].as<double>();
    cfg.yaw_rate_limit     = node["yaw_rate_limit"].as<double>();
    cfg.slip_angle_limit   = node["slip_angle_limit"].as<double>();
    cfg.stop_speed_epsilon = node["stop_speed_epsilon"].as<double>();
    cfg.validate();
    return cfg;
}

fs::path resolve_path(const fs::path& path)
{
    if (path.is_absolute()) {
        return path;
    }
    fs::path parameters_root = fs::path(COMMONROAD_PARAM_ROOT);
    return parameters_root.parent_path() / "config" / path;
}

} // namespace

LowSpeedSafetyConfig load_low_speed_safety_config(const std::filesystem::path& path)
{
    YAML::Node node = YAML::LoadFile(resolve_path(path).string());
    return parse_low_speed_config(node);
}

LowSpeedSafetyConfig load_default_low_speed_safety_config()
{
    return load_low_speed_safety_config("low_speed_safety.yaml");
}

} // namespace vehiclemodels::sim
