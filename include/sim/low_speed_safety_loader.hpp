#pragma once

#include <filesystem>

#include "sim/low_speed_safety.hpp"

namespace vehiclemodels::sim {

LowSpeedSafetyConfig load_low_speed_safety_config(const std::filesystem::path& path);
LowSpeedSafetyConfig load_default_low_speed_safety_config();

} // namespace vehiclemodels::sim
