#pragma once

#include <filesystem>

#include "sim/low_speed_safety.hpp"
#include "sim/model_timing.hpp"

namespace vehiclemodels::sim {

LowSpeedSafetyConfig load_low_speed_safety_config(const std::filesystem::path& path);
LowSpeedSafetyConfig load_default_low_speed_safety_config();
LowSpeedSafetyConfig load_low_speed_safety_config_for_model(ModelType model);

} // namespace vehiclemodels::sim
