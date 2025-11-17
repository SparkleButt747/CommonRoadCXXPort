#include "simulation/simulation_daemon.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "models/vehicle_dynamics_ks_cog.hpp"
#include "models/vehiclemodels/init_kst.hpp"
#include "models/vehiclemodels/init_mb.hpp"
#include "models/vehiclemodels/init_st.hpp"
#include "models/vehiclemodels/init_std.hpp"
#include "models/vehiclemodels/init_ks.hpp"
#include "models/vehiclemodels/vehicle_dynamics_ks.hpp"
#include "models/vehiclemodels/vehicle_dynamics_kst.hpp"
#include "models/vehiclemodels/vehicle_dynamics_mb.hpp"
#include "models/vehiclemodels/vehicle_dynamics_st.hpp"
#include "models/vehiclemodels/vehicle_dynamics_std.hpp"
#include "vehicle_parameters.hpp"

namespace velox::simulation {

UserInput UserInput::clamped(const UserInputLimits& limits) const
{
    validate(limits);
    UserInput copy = *this;
    copy.longitudinal.throttle = std::clamp(copy.longitudinal.throttle, limits.min_throttle, limits.max_throttle);
    copy.longitudinal.brake    = std::clamp(copy.longitudinal.brake, limits.min_brake, limits.max_brake);
    copy.steering_nudge        = std::clamp(copy.steering_nudge, limits.min_steering_nudge, limits.max_steering_nudge);
    return copy;
}

void UserInput::validate(const UserInputLimits& limits) const
{
    const auto require_finite = [](double value, const char* field) {
        if (!std::isfinite(value)) {
            std::ostringstream oss;
            oss << "UserInput." << field << " must be finite; got " << value;
            throw std::invalid_argument(oss.str());
        }
    };

    const auto require_in_range = [](double value, const char* field, double min, double max) {
        if (value < min || value > max) {
            std::ostringstream oss;
            oss << "UserInput." << field << " of " << value << " outside [" << min << ", " << max << "]";
            throw std::invalid_argument(oss.str());
        }
    };

    require_finite(timestamp, "timestamp");
    if (timestamp < 0.0) {
        std::ostringstream oss;
        oss << "UserInput.timestamp must be non-negative; got " << timestamp;
        throw std::invalid_argument(oss.str());
    }

    require_finite(dt, "dt");
    if (dt <= 0.0) {
        std::ostringstream oss;
        oss << "UserInput.dt must be positive; got " << dt;
        throw std::invalid_argument(oss.str());
    }

    require_finite(longitudinal.throttle, "longitudinal.throttle");
    require_in_range(longitudinal.throttle,
                     "longitudinal.throttle",
                     limits.min_throttle,
                     limits.max_throttle);

    require_finite(longitudinal.brake, "longitudinal.brake");
    require_in_range(longitudinal.brake, "longitudinal.brake", limits.min_brake, limits.max_brake);

    require_finite(steering_nudge, "steering_nudge");
    require_in_range(steering_nudge,
                     "steering_nudge",
                     limits.min_steering_nudge,
                     limits.max_steering_nudge);

    switch (gear) {
        case GearSelection::Park:
        case GearSelection::Reverse:
        case GearSelection::Neutral:
        case GearSelection::Drive:
            break;
        default:
            throw std::invalid_argument("UserInput.gear set to unknown value");
    }
}

namespace {
ModelInterface build_model_interface(ModelType model)
{
    ModelInterface iface{};
    switch (model) {
        case ModelType::KS_REAR:
            iface.init_fn = [](const std::vector<double>& init_state, const models::VehicleParameters&) {
                return models::init_ks(init_state);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const models::VehicleParameters& params,
                                   double) {
                return models::vehicle_dynamics_ks(x, u, params);
            };
            iface.speed_fn = [](const std::vector<double>& state, const models::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;

        case ModelType::KS_COG:
            iface.init_fn = [](const std::vector<double>& init_state, const models::VehicleParameters&) {
                return models::init_ks(init_state);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const models::VehicleParameters& params,
                                   double) {
                std::array<double, 5> x_arr{};
                std::array<double, 2> u_arr{};
                for (std::size_t i = 0; i < x_arr.size(); ++i) {
                    x_arr[i] = (i < x.size()) ? x[i] : 0.0;
                }
                if (!u.empty()) {
                    u_arr[0] = u[0];
                }
                if (u.size() > 1) {
                    u_arr[1] = u[1];
                }
                auto result = models::utils::vehicle_dynamics_ks_cog(x_arr, u_arr, params);
                return std::vector<double>(result.begin(), result.end());
            };
            iface.speed_fn = [](const std::vector<double>& state, const models::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;

        case ModelType::KST:
            iface.init_fn = [](const std::vector<double>& init_state, const models::VehicleParameters&) {
                std::vector<double> core(5, 0.0);
                const std::size_t copy = std::min<std::size_t>(core.size(), init_state.size());
                std::copy_n(init_state.begin(), copy, core.begin());
                const double alpha0 = (init_state.size() > 5) ? init_state[5] : 0.0;
                return models::init_kst(core, alpha0);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const models::VehicleParameters& params,
                                   double) {
                return models::vehicle_dynamics_kst(x, u, params);
            };
            iface.speed_fn = [](const std::vector<double>& state, const models::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;

        case ModelType::MB:
            iface.init_fn = [](const std::vector<double>& init_state, const models::VehicleParameters& params) {
                return models::init_mb(init_state, params);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const models::VehicleParameters& params,
                                   double) {
                return models::vehicle_dynamics_mb(x, u, params);
            };
            iface.speed_fn = [](const std::vector<double>& state, const models::VehicleParameters&) {
                if (state.size() > 10) {
                    return std::hypot(state[3], state[10]);
                }
                if (state.size() > 3) {
                    return std::abs(state[3]);
                }
                return 0.0;
            };
            break;

        case ModelType::ST:
            iface.init_fn = [](const std::vector<double>& init_state, const models::VehicleParameters&) {
                return models::init_st(init_state);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const models::VehicleParameters& params,
                                   double) {
                return models::vehicle_dynamics_st(x, u, params);
            };
            iface.speed_fn = [](const std::vector<double>& state, const models::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;

        case ModelType::STD:
            iface.init_fn = [](const std::vector<double>& init_state, const models::VehicleParameters& params) {
                return models::init_std(init_state, params);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const models::VehicleParameters& params,
                                   double dt) {
                return models::vehicle_dynamics_std(x, u, params, dt);
            };
            iface.speed_fn = [](const std::vector<double>& state, const models::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;
    }

    if (!iface.valid()) {
        throw std::runtime_error("Unsupported model type for simulator");
    }
    return iface;
}

LowSpeedSafety build_low_speed_safety(ModelType model,
                                      const models::VehicleParameters& params,
                                      const LowSpeedSafetyConfig& cfg)
{
    std::optional<int> longitudinal;
    std::optional<int> lateral;
    std::optional<int> yaw_rate;
    std::optional<int> slip;
    std::vector<int>   wheel_indices;
    std::optional<int> steering;

    switch (model) {
        case ModelType::KS_REAR:
        case ModelType::KS_COG:
        case ModelType::KST:
            longitudinal = 3;
            steering     = 2;
            break;

        case ModelType::ST:
            longitudinal = 3;
            yaw_rate     = 5;
            slip         = 6;
            steering     = 2;
            break;

        case ModelType::STD:
            longitudinal = 3;
            yaw_rate     = 5;
            slip         = 6;
            wheel_indices = {7, 8};
            steering      = 2;
            break;

        case ModelType::MB:
            longitudinal = 3;
            lateral      = 10;
            yaw_rate     = 5;
            wheel_indices = {23, 24, 25, 26};
            steering      = 2;
            break;
    }

    std::optional<double> wheelbase;
    std::optional<double> rear_length;
    if (std::isfinite(params.a) && std::isfinite(params.b)) {
        const double wb = params.a + params.b;
        if (wb > 0.0) {
            wheelbase = wb;
        }
    }
    if (std::isfinite(params.b) && params.b > 0.0) {
        rear_length = params.b;
    }

    return LowSpeedSafety(cfg,
                          longitudinal,
                          lateral,
                          yaw_rate,
                          slip,
                          wheel_indices,
                          steering,
                          wheelbase,
                          rear_length);
}

std::vector<double> seed_state(const ModelInterface& model_if, const models::VehicleParameters& params, const std::vector<double>& user)
{
    if (user.empty()) {
        return model_if.init_fn({}, params);
    }
    return model_if.init_fn(user, params);
}

} // namespace

SimulationDaemon::SimulationDaemon(const InitParams& init)
    : init_(init)
    , configs_(init.config_root, init.parameter_root)
    , model_(init.model)
{
    load_vehicle_parameters(init.vehicle_id);

    // Initial wiring occurs during construction
    const auto timing = configs_.load_model_timing(model_);
    const auto default_state = build_model_interface(model_).init_fn({}, params_);
    ResetParams reset_params{};
    reset_params.initial_state = default_state;
    reset_params.dt            = timing.nominal_dt;
    reset(reset_params);
}

void SimulationDaemon::reset(const ResetParams& params)
{
    if (params.model) {
        model_ = *params.model;
    }
    if (params.vehicle_id) {
        init_.vehicle_id = *params.vehicle_id;
        load_vehicle_parameters(*params.vehicle_id);
    }

    model_interface_ = build_model_interface(model_);

    rebuild_controllers();
    rebuild_safety();

    const auto timing = configs_.load_model_timing(model_);
    const double dt = params.dt.value_or(timing.nominal_dt);
    const auto initial = seed_state(model_interface_, params_, params.initial_state);

    rebuild_simulator(dt, initial);

    const double initial_angle = (initial.size() > 2) ? initial[2] : 0.0;
    if (steering_wheel_) {
        steering_wheel_->reset(initial_angle);
    }
    if (final_steer_) {
        final_steer_->reset(initial_angle);
    }
    if (accel_controller_) {
        accel_controller_->reset();
    }

    cumulative_distance_m_ = 0.0;
    cumulative_energy_j_   = 0.0;
    last_telemetry_        = {};
}

telemetry::SimulationTelemetry SimulationDaemon::step(const UserInput& input)
{
    if (!simulator_ || !accel_controller_ || !steering_wheel_ || !final_steer_ || !safety_) {
        throw std::runtime_error("SimulationDaemon not initialized");
    }

    const auto sanitized_input = input.clamped(kDefaultUserInputLimits);
    const double dt            = sanitized_input.dt;

    const double start_speed   = simulator_->speed();
    const double current_angle = (simulator_->state().size() > 2) ? simulator_->state()[2] : 0.0;
    const auto steering_output = steering_wheel_->update(sanitized_input.steering_nudge, dt);
    const auto final_output = final_steer_->update(steering_output.target_angle, current_angle, dt);

    const double speed = simulator_->speed();
    const auto accel_output = accel_controller_->step(sanitized_input.longitudinal, speed, dt);

    std::vector<double> control{final_output.rate, accel_output.acceleration};
    simulator_->set_dt(dt);
    simulator_->step(control);

    const double end_speed = simulator_->speed();
    cumulative_distance_m_ += 0.5 * (std::abs(start_speed) + std::abs(end_speed)) * dt;
    cumulative_energy_j_   += accel_output.battery_power * dt;

    last_telemetry_ = compute_telemetry(accel_output, steering_output, final_output);
    return last_telemetry_;
}

std::vector<telemetry::SimulationTelemetry> SimulationDaemon::step(const std::vector<UserInput>& batch_inputs)
{
    std::vector<telemetry::SimulationTelemetry> telemetry;
    telemetry.reserve(batch_inputs.size());
    for (const auto& input : batch_inputs) {
        telemetry.push_back(step(input));
    }
    return telemetry;
}

SimulationSnapshot SimulationDaemon::snapshot() const
{
    SimulationSnapshot snap{};
    snap.telemetry = last_telemetry_;
    if (simulator_) {
        snap.state = simulator_->state();
        snap.dt    = simulator_->dt();
    }
    return snap;
}

void SimulationDaemon::load_vehicle_parameters(int vehicle_id)
{
    params_ = configs_.load_vehicle_parameters(vehicle_id);
}

void SimulationDaemon::rebuild_controllers()
{
    const auto steering_cfg = configs_.load_steering_config();
    steering_wheel_.emplace(steering_cfg.wheel, params_.steering);
    final_steer_.emplace(steering_cfg.final, params_.steering);

    const auto aero_cfg         = configs_.load_aero_config();
    const auto rolling_cfg      = configs_.load_rolling_resistance_config();
    const auto brake_cfg        = configs_.load_brake_config();
    const auto powertrain_cfg   = configs_.load_powertrain_config();
    const auto accel_controller = configs_.load_final_accel_controller_config();

    accel_controller_.emplace(params_.m,
                              params_.R_w,
                              powertrain_cfg,
                              aero_cfg,
                              rolling_cfg,
                              brake_cfg,
                              accel_controller);
}

void SimulationDaemon::rebuild_safety()
{
    const auto safety_cfg = configs_.load_low_speed_safety_config(model_);
    safety_.emplace(build_low_speed_safety(model_, params_, safety_cfg));
}

void SimulationDaemon::rebuild_simulator(double dt, const std::vector<double>& initial_state)
{
    if (!safety_) {
        throw std::runtime_error("Safety system not initialized");
    }
    simulator_ = std::make_unique<VehicleSimulator>(model_interface_, params_, dt, *safety_);
    simulator_->reset(initial_state);
}

telemetry::SimulationTelemetry SimulationDaemon::compute_telemetry(
    const controllers::longitudinal::ControllerOutput& accel_output,
    const controllers::SteeringWheel::Output& steering_input,
    const controllers::FinalSteerController::Output& steering_output) const
{
    const auto* simulator_ptr = simulator_.get();
    const auto* safety_ptr    = simulator_ptr ? &simulator_ptr->safety() : safety_ ? &(*safety_) : nullptr;
    const auto& state         = simulator_ptr ? simulator_ptr->state() : std::vector<double>{};
    const double measured_speed = simulator_ptr ? simulator_ptr->speed() : 0.0;

    return telemetry::compute_simulation_telemetry(model_,
                                                   params_,
                                                   state,
                                                   accel_output,
                                                   steering_input,
                                                   steering_output,
                                                   safety_ptr,
                                                   measured_speed,
                                                   cumulative_distance_m_,
                                                   cumulative_energy_j_);
}

} // namespace velox::simulation

