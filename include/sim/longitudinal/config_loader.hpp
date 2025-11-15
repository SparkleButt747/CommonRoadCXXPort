#pragma once

#include <filesystem>
#include <string>

#include "sim/longitudinal/aero.hpp"
#include "sim/longitudinal/brake.hpp"
#include "sim/longitudinal/final_accel_controller.hpp"
#include "sim/longitudinal/powertrain.hpp"
#include "sim/longitudinal/rolling_resistance.hpp"

namespace vehiclemodels::sim::longitudinal {

AeroConfig load_aero_config(const std::filesystem::path& path);
RollingResistanceConfig load_rolling_resistance_config(const std::filesystem::path& path);
BrakeConfig load_brake_config(const std::filesystem::path& path);
PowertrainConfig load_powertrain_config(const std::filesystem::path& path);
FinalAccelControllerConfig load_final_accel_controller_config(const std::filesystem::path& path);

AeroConfig load_default_aero_config();
RollingResistanceConfig load_default_rolling_resistance_config();
BrakeConfig load_default_brake_config();
PowertrainConfig load_default_powertrain_config();
FinalAccelControllerConfig load_default_final_accel_controller_config();

} // namespace vehiclemodels::sim::longitudinal
