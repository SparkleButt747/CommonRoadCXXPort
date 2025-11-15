#pragma once

#include <array>
#include <cstddef>

namespace vehiclemodels::sim {

enum class ModelType {
    KS_REAR = 0,
    KS_COG  = 1,
    KST     = 2,
    MB      = 3,
    ST      = 4,
    STD     = 5
};

struct ModelTimingInfo {
    float nominal_dt = 0.0f;
    float max_dt     = 0.0f;
};

inline constexpr float kMinStableDt = 0.001f;

namespace detail {
inline constexpr std::array<ModelTimingInfo, 6> kModelTimings = {
    ModelTimingInfo{0.010f, 0.050f}, // KS_REAR
    ModelTimingInfo{0.010f, 0.050f}, // KS_COG
    ModelTimingInfo{0.010f, 0.030f}, // KST
    ModelTimingInfo{0.005f, 0.005f}, // MB
    ModelTimingInfo{0.010f, 0.020f}, // ST
    ModelTimingInfo{0.010f, 0.010f}  // STD
};

inline constexpr std::array<const char*, 6> kModelNames = {
    "KS (vehicle_dynamics_ks)",
    "KS CoG (vehicle_dynamics_ks_cog)",
    "KST trailer (vehicle_dynamics_kst)",
    "Multi-body (vehicle_dynamics_mb)",
    "ST (vehicle_dynamics_st)",
    "STD (vehicle_dynamics_std)"
};
} // namespace detail

inline constexpr const ModelTimingInfo& model_timing(ModelType type)
{
    const std::size_t index = static_cast<std::size_t>(type);
    return detail::kModelTimings[index];
}

inline constexpr const char* model_display_name(ModelType type)
{
    const std::size_t index = static_cast<std::size_t>(type);
    return detail::kModelNames[index];
}

} // namespace vehiclemodels::sim
