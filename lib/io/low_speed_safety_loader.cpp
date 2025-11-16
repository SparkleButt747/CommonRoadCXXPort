#include "io/low_speed_safety_loader.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#ifndef VELOX_PARAM_ROOT
#    error "VELOX_PARAM_ROOT must be defined"
#endif

namespace velox::simulation {

namespace {
namespace fs = std::filesystem;

const char* model_suffix(ModelType model)
{
    switch (model) {
        case ModelType::KS_REAR: return "ks";
        case ModelType::KS_COG:  return "ks_cog";
        case ModelType::KST:     return "kst";
        case ModelType::MB:      return "mb";
        case ModelType::ST:      return "st";
        case ModelType::STD:     return "std";
    }
    return "";
}

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
    fs::path parameters_root = fs::path(VELOX_PARAM_ROOT);
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

LowSpeedSafetyConfig load_low_speed_safety_config_for_model(ModelType model)
{
    const std::string override_filename =
        std::string("low_speed_safety_") + model_suffix(model) + ".yaml";
    const fs::path override_path = resolve_path(override_filename);
    if (!override_filename.empty() && fs::exists(override_path)) {
        YAML::Node node = YAML::LoadFile(override_path.string());
        return parse_low_speed_config(node);
    }
    return load_default_low_speed_safety_config();
}

} // namespace velox::simulation
