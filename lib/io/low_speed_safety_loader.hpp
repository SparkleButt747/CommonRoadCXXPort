#pragma once

#include <filesystem>

#include "simulation/low_speed_safety.hpp"
#include "simulation/model_timing.hpp"

namespace velox::simulation {

LowSpeedSafetyConfig load_low_speed_safety_config(const std::filesystem::path& path);
LowSpeedSafetyConfig load_default_low_speed_safety_config();
LowSpeedSafetyConfig load_low_speed_safety_config_for_model(ModelType model);

} // namespace velox::simulation
