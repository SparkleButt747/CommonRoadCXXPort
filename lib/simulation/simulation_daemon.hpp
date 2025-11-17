#pragma once

#include <filesystem>
#include <optional>
#include <vector>

#include "controllers/longitudinal/final_accel_controller.hpp"
#include "controllers/steering_controller.hpp"
#include "io/config_manager.hpp"
#include "simulation/low_speed_safety.hpp"
#include "simulation/model_timing.hpp"
#include "simulation/vehicle_simulator.hpp"
#include "telemetry/telemetry.hpp"

namespace velox::simulation {

enum class GearSelection {
    Park,
    Reverse,
    Neutral,
    Drive,
};

struct UserInputLimits {
    double min_throttle{0.0};
    double max_throttle{1.0};
    double min_brake{0.0};
    double max_brake{1.0};
    double min_steering_nudge{-1.0};
    double max_steering_nudge{1.0};
};

struct UserInput {
    controllers::longitudinal::DriverIntent longitudinal{};
    double                                  steering_nudge = 0.0;
    GearSelection                           gear{GearSelection::Drive};
    double                                  timestamp{0.0};
    double                                  dt{0.0};

    [[nodiscard]] UserInput clamped(const UserInputLimits& limits = {}) const;
    void validate(const UserInputLimits& limits = {}) const;
};

inline constexpr UserInputLimits kDefaultUserInputLimits{};

struct ResetParams {
    std::optional<ModelType> model{};
    std::optional<int>       vehicle_id{};
    std::vector<double>      initial_state{};
    std::optional<double>    dt{};
};

struct SimulationSnapshot {
    std::vector<double>               state{};
    telemetry::SimulationTelemetry    telemetry{};
    double                            dt{0.0};
};

class SimulationDaemon {
public:
    struct InitParams {
        ModelType                 model{ModelType::KS_REAR};
        int                       vehicle_id{1};
        std::filesystem::path     config_root{};
        std::filesystem::path     parameter_root{};
    };

    explicit SimulationDaemon(const InitParams& init);

    void reset(const ResetParams& params);

    telemetry::SimulationTelemetry step(const UserInput& input);
    std::vector<telemetry::SimulationTelemetry> step(const std::vector<UserInput>& batch_inputs);

    [[nodiscard]] const VehicleSimulator* simulator() const { return simulator_.get(); }
    [[nodiscard]] const controllers::longitudinal::FinalAccelController* accel_controller() const
    {
        return accel_controller_ ? &(*accel_controller_) : nullptr;
    }
    [[nodiscard]] const controllers::FinalSteerController* steering_controller() const
    {
        return final_steer_ ? &(*final_steer_) : nullptr;
    }
    [[nodiscard]] const controllers::SteeringWheel* steering_wheel() const
    {
        return steering_wheel_ ? &(*steering_wheel_) : nullptr;
    }
    [[nodiscard]] const models::VehicleParameters& vehicle_parameters() const { return params_; }
    [[nodiscard]] ModelType model() const { return model_; }
    [[nodiscard]] const telemetry::SimulationTelemetry& telemetry() const { return last_telemetry_; }

    [[nodiscard]] SimulationSnapshot snapshot() const;

private:
    InitParams              init_{};
    io::ConfigManager       configs_{};
    ModelType               model_{};

    models::VehicleParameters                              params_{};
    ModelInterface                                          model_interface_{};
    std::optional<LowSpeedSafety>                          safety_{};
    std::unique_ptr<VehicleSimulator>                      simulator_{};

    std::optional<controllers::SteeringWheel>             steering_wheel_{};
    std::optional<controllers::FinalSteerController>       final_steer_{};
    std::optional<controllers::longitudinal::FinalAccelController> accel_controller_{};

    double cumulative_distance_m_{0.0};
    double cumulative_energy_j_{0.0};

    telemetry::SimulationTelemetry last_telemetry_{};

    void load_vehicle_parameters(int vehicle_id);
    void rebuild_controllers();
    void rebuild_safety();
    void rebuild_simulator(double dt, const std::vector<double>& initial_state);
    telemetry::SimulationTelemetry compute_telemetry(
        const controllers::longitudinal::ControllerOutput& accel_output,
        const controllers::SteeringWheel::Output& steering_input,
        const controllers::FinalSteerController::Output&    steering_output) const;
};

} // namespace velox::simulation

