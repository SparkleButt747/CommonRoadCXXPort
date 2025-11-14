#include "sim/longitudinal/config_loader.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#ifndef COMMONROAD_PARAM_ROOT
#    error "COMMONROAD_PARAM_ROOT must be defined"
#endif

namespace vehiclemodels::sim::longitudinal {

namespace {
namespace fs = std::filesystem;

template <typename T>
T get_required_scalar(const YAML::Node& node, const char* key)
{
    auto value = node[key];
    if (!value) {
        throw std::runtime_error(std::string{"missing key: "} + key);
    }
    return value.as<T>();
}

fs::path config_root()
{
    fs::path parameters_root = fs::path(COMMONROAD_PARAM_ROOT);
    return parameters_root.parent_path() / "config";
}

AeroConfig parse_aero_config(const YAML::Node& node)
{
    AeroConfig cfg{};
    cfg.drag_coefficient      = get_required_scalar<double>(node, "drag_coefficient");
    cfg.downforce_coefficient = get_required_scalar<double>(node, "downforce_coefficient");
    cfg.validate();
    return cfg;
}

RollingResistanceConfig parse_rolling_config(const YAML::Node& node)
{
    RollingResistanceConfig cfg{};
    cfg.c_rr = get_required_scalar<double>(node, "c_rr");
    cfg.validate();
    return cfg;
}

BrakeConfig parse_brake_config(const YAML::Node& node)
{
    BrakeConfig cfg{};
    cfg.max_force       = get_required_scalar<double>(node, "max_force");
    cfg.max_regen_force = get_required_scalar<double>(node, "max_regen_force");
    cfg.min_regen_speed = get_required_scalar<double>(node, "min_regen_speed");
    cfg.validate();
    return cfg;
}

PowertrainConfig parse_powertrain_config(const YAML::Node& node)
{
    PowertrainConfig cfg{};
    cfg.max_drive_torque     = get_required_scalar<double>(node, "max_drive_torque");
    cfg.max_regen_torque     = get_required_scalar<double>(node, "max_regen_torque");
    cfg.max_power            = get_required_scalar<double>(node, "max_power");
    cfg.drive_efficiency     = get_required_scalar<double>(node, "drive_efficiency");
    cfg.regen_efficiency     = get_required_scalar<double>(node, "regen_efficiency");
    cfg.min_soc              = get_required_scalar<double>(node, "min_soc");
    cfg.max_soc              = get_required_scalar<double>(node, "max_soc");
    cfg.initial_soc          = get_required_scalar<double>(node, "initial_soc");
    cfg.battery_capacity_kwh = get_required_scalar<double>(node, "battery_capacity_kwh");
    cfg.validate();
    return cfg;
}

FinalAccelControllerConfig parse_final_controller_config(const YAML::Node& node)
{
    FinalAccelControllerConfig cfg{};
    cfg.tau_throttle       = get_required_scalar<double>(node, "tau_throttle");
    cfg.tau_brake          = get_required_scalar<double>(node, "tau_brake");
    cfg.accel_min          = get_required_scalar<double>(node, "accel_min");
    cfg.accel_max          = get_required_scalar<double>(node, "accel_max");
    if (auto stop = node["stop_speed_epsilon"]) {
        cfg.stop_speed_epsilon = stop.as<double>();
    }
    cfg.validate();
    return cfg;
}

fs::path resolve_path(const fs::path& path)
{
    if (path.is_absolute()) {
        return path;
    }
    return config_root() / path;
}

} // namespace

AeroConfig load_aero_config(const std::filesystem::path& path)
{
    YAML::Node node = YAML::LoadFile(resolve_path(path).string());
    return parse_aero_config(node);
}

RollingResistanceConfig load_rolling_resistance_config(const std::filesystem::path& path)
{
    YAML::Node node = YAML::LoadFile(resolve_path(path).string());
    return parse_rolling_config(node);
}

BrakeConfig load_brake_config(const std::filesystem::path& path)
{
    YAML::Node node = YAML::LoadFile(resolve_path(path).string());
    return parse_brake_config(node);
}

PowertrainConfig load_powertrain_config(const std::filesystem::path& path)
{
    YAML::Node node = YAML::LoadFile(resolve_path(path).string());
    return parse_powertrain_config(node);
}

FinalAccelControllerConfig load_final_accel_controller_config(const std::filesystem::path& path)
{
    YAML::Node node = YAML::LoadFile(resolve_path(path).string());
    return parse_final_controller_config(node);
}

AeroConfig load_default_aero_config()
{
    return load_aero_config("aero.yaml");
}

RollingResistanceConfig load_default_rolling_resistance_config()
{
    return load_rolling_resistance_config("rolling.yaml");
}

BrakeConfig load_default_brake_config()
{
    return load_brake_config("brakes.yaml");
}

PowertrainConfig load_default_powertrain_config()
{
    return load_powertrain_config("powertrain.yaml");
}

FinalAccelControllerConfig load_default_final_accel_controller_config()
{
    return load_final_accel_controller_config("final_accel_controller.yaml");
}

} // namespace vehiclemodels::sim::longitudinal
