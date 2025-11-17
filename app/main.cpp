// app/main.cpp
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// SDL2
#if defined(__has_include)
#  if __has_include(<SDL2/SDL.h>)
#    include <SDL2/SDL.h>
#  else
#    include <SDL.h>
#  endif
#else
#  include <SDL.h>
#endif

// ImGui core + SDL2/SDL_Renderer2 backends
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

// Vehicle params and models
#include "vehicle_parameters.hpp"

#include "vehicle/parameters_vehicle1.hpp"
#include "vehicle/parameters_vehicle2.hpp"
#include "vehicle/parameters_vehicle3.hpp"
#include "vehicle/parameters_vehicle4.hpp"

#include "models/vehiclemodels/vehicle_dynamics_ks.hpp"
#include "models/vehiclemodels/vehicle_dynamics_kst.hpp"
#include "models/vehiclemodels/vehicle_dynamics_mb.hpp"
#include "models/vehiclemodels/vehicle_dynamics_st.hpp"
#include "models/vehiclemodels/vehicle_dynamics_std.hpp"
#include "models/vehicle_dynamics_ks_cog.hpp"
#include "controllers/steering_controller.hpp"

#include "models/vehiclemodels/init_ks.hpp"
#include "models/vehiclemodels/init_kst.hpp"
#include "models/vehiclemodels/init_mb.hpp"
#include "models/vehiclemodels/init_st.hpp"
#include "models/vehiclemodels/init_std.hpp"

#include "controllers/longitudinal/final_accel_controller.hpp"
#include "io/config_manager.hpp"
#include "simulation/model_timing.hpp"
#include "simulation/vehicle_simulator.hpp"

namespace vm     = velox::models;
namespace vutils = velox::controllers;
namespace longi  = velox::controllers::longitudinal;
namespace vsim   = velox::simulation;
namespace vio    = velox::io;

using ModelType = vsim::ModelType;

struct Simulation {
    ModelType                               model;
    int                                     vehicle_id;
    vm::VehicleParameters                   params;
    std::vector<double>                     x;     // state
    std::vector<double>                     x0;    // initial state
    std::vector<double>                     u;     // input [steer_rate, accel]
    float                                   dt;    // integration step [s]
    std::unique_ptr<vsim::VehicleSimulator> integrator;
};

static vsim::ModelInterface build_model_interface(ModelType model);
static vsim::LowSpeedSafety build_low_speed_safety(ModelType model,
                                                   const vm::VehicleParameters& params,
                                                   const vsim::LowSpeedSafetyConfig& cfg);

static constexpr float kSliderMinDt = vsim::kMinStableDt;
static constexpr float kSliderMaxDt = 0.05f;
static constexpr double kDtRebuildRatio = 2.0;

static bool enforce_model_dt_limits(Simulation& sim,
                                    ModelType model,
                                    const vio::ConfigManager& configs)
{
    const auto timing = configs.load_model_timing(model);
    const float clamped = std::clamp(sim.dt, kSliderMinDt, timing.max_dt);
    const bool changed  = std::abs(clamped - sim.dt) > 1e-9f;
    if (changed) {
        sim.dt = clamped;
    }
    return changed;
}

static void reconfigure_simulator_dt(Simulation& sim,
                                     const vsim::LowSpeedSafetyConfig& safety_cfg)
{
    if (!sim.integrator) {
        return;
    }

    const double new_dt     = static_cast<double>(sim.dt);
    const double current_dt = sim.integrator->dt();
    const double diff       = std::abs(current_dt - new_dt);
    if (diff < 1e-9) {
        return;
    }

    const double ratio = (current_dt > 0.0)
        ? std::max(current_dt, new_dt) / std::min(current_dt, new_dt)
        : std::numeric_limits<double>::infinity();

    if (ratio >= kDtRebuildRatio) {
        std::vector<double> snapshot = sim.integrator->state();
        auto iface  = build_model_interface(sim.model);
        auto safety = build_low_speed_safety(sim.model, sim.params, safety_cfg);
        auto new_integrator = std::make_unique<vsim::VehicleSimulator>(
            std::move(iface),
            sim.params,
            new_dt,
            std::move(safety));
        new_integrator->seed_state(snapshot);
        sim.integrator = std::move(new_integrator);
    } else {
        sim.integrator->set_dt(new_dt);
    }

    sim.x = sim.integrator->state();
}

static void set_dt_warning(std::string& message,
                           double& expires_at,
                           double now_seconds,
                           ModelType model,
                           float requested_dt,
                           float safe_max_dt)
{
    std::ostringstream oss;
    oss << vsim::model_display_name(model) << " is validated up to "
        << std::fixed << std::setprecision(3) << safe_max_dt
        << " s; clamped requested " << requested_dt << " s to stay stable.";
    message     = oss.str();
    expires_at  = now_seconds + 4.0;
}

// load parameters by id
static vm::VehicleParameters load_vehicle_params(int id, const std::string& root = {})
{
    switch (id) {
        case 1: return vm::parameters_vehicle1(root);
        case 2: return vm::parameters_vehicle2(root);
        case 3: return vm::parameters_vehicle3(root);
        case 4: return vm::parameters_vehicle4(root);
        default:
            throw std::runtime_error("Unsupported vehicle_id (expected 1..4)");
    }
}

static vsim::ModelInterface build_model_interface(ModelType model)
{
    vsim::ModelInterface iface{};
    switch (model) {
        case ModelType::KS_REAR:
            iface.init_fn = [](const std::vector<double>& init_state,
                               const vm::VehicleParameters&) {
                return vm::init_ks(init_state);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const vm::VehicleParameters& params,
                                   double) {
                return vm::vehicle_dynamics_ks(x, u, params);
            };
            iface.speed_fn = [](const std::vector<double>& state,
                                const vm::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;

        case ModelType::KS_COG:
            iface.init_fn = [](const std::vector<double>& init_state,
                               const vm::VehicleParameters&) {
                return vm::init_ks(init_state);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const vm::VehicleParameters& params,
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
                auto result = vm::utils::vehicle_dynamics_ks_cog(x_arr, u_arr, params);
                return std::vector<double>(result.begin(), result.end());
            };
            iface.speed_fn = [](const std::vector<double>& state,
                                const vm::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;

        case ModelType::KST:
            iface.init_fn = [](const std::vector<double>& init_state,
                               const vm::VehicleParameters&) {
                std::vector<double> core(5, 0.0);
                const std::size_t copy = std::min<std::size_t>(core.size(), init_state.size());
                std::copy_n(init_state.begin(), copy, core.begin());
                const double alpha0 = (init_state.size() > 5) ? init_state[5] : 0.0;
                return vm::init_kst(core, alpha0);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const vm::VehicleParameters& params,
                                   double) {
                return vm::vehicle_dynamics_kst(x, u, params);
            };
            iface.speed_fn = [](const std::vector<double>& state,
                                const vm::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;

        case ModelType::MB:
            iface.init_fn = [](const std::vector<double>& init_state,
                               const vm::VehicleParameters& params) {
                return vm::init_mb(init_state, params);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const vm::VehicleParameters& params,
                                   double) {
                return vm::vehicle_dynamics_mb(x, u, params);
            };
            iface.speed_fn = [](const std::vector<double>& state,
                                const vm::VehicleParameters&) {
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
            iface.init_fn = [](const std::vector<double>& init_state,
                               const vm::VehicleParameters&) {
                return vm::init_st(init_state);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const vm::VehicleParameters& params,
                                   double) {
                return vm::vehicle_dynamics_st(x, u, params);
            };
            iface.speed_fn = [](const std::vector<double>& state,
                                const vm::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;

        case ModelType::STD:
            iface.init_fn = [](const std::vector<double>& init_state,
                               const vm::VehicleParameters& params) {
                return vm::init_std(init_state, params);
            };
            iface.dynamics_fn = [](const std::vector<double>& x,
                                   const std::vector<double>& u,
                                   const vm::VehicleParameters& params,
                                   double dt) {
                return vm::vehicle_dynamics_std(x, u, params, dt);
            };
            iface.speed_fn = [](const std::vector<double>& state,
                                const vm::VehicleParameters&) {
                return (state.size() > 3) ? std::abs(state[3]) : 0.0;
            };
            break;
    }

    if (!iface.valid()) {
        throw std::runtime_error("Unsupported model type for simulator");
    }
    return iface;
}

static vsim::LowSpeedSafety build_low_speed_safety(ModelType model,
                                                   const vm::VehicleParameters& params,
                                                   const vsim::LowSpeedSafetyConfig& cfg)
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

    return vsim::LowSpeedSafety(cfg,
                                longitudinal,
                                lateral,
                                yaw_rate,
                                slip,
                                wheel_indices,
                                steering,
                                wheelbase,
                                rear_length);
}

// compute RHS for current model
// --------------------------------------------------------------
// Telemetry
// --------------------------------------------------------------

struct Telemetry {
    double speed          = 0.0;
    double v_long         = 0.0;
    double v_lat          = 0.0;
    double v_global_x     = 0.0;
    double v_global_y     = 0.0;
    double a_long         = 0.0;
    double a_lat          = 0.0;
    bool   low_speed_engaged = false;
    double soc            = 0.0;
};

static Telemetry compute_telemetry(const Simulation& sim,
                                   const longi::ControllerOutput& accel_output,
                                   const longi::FinalAccelController* accel_controller)
{
    Telemetry t{};

    const auto* simulator = sim.integrator.get();
    const auto& state     = simulator ? simulator->state() : sim.x;

    const double wheelbase = sim.params.a + sim.params.b;
    const double delta     = (state.size() > 2) ? state[2] : 0.0;

    double v_long = 0.0;
    double v_lat  = 0.0;
    double yaw    = 0.0;
    double beta   = 0.0;

    switch (sim.model) {
        case ModelType::KS_REAR:
        case ModelType::KS_COG:
        case ModelType::KST:
            if (state.size() >= 5) {
                v_long = state[3];
                yaw    = state[4];
            }
            beta = 0.0;
            break;

        case ModelType::ST:
            if (state.size() >= 7) {
                v_long = state[3];
                yaw    = state[4];
                beta   = state[6];
            }
            break;

        case ModelType::STD:
            if (state.size() >= 9) {
                v_long = state[3];
                yaw    = state[4];
                beta   = state[6];
            }
            break;

        case ModelType::MB:
            if (state.size() >= 11) {
                v_long = state[3];
                v_lat  = state[10];
                yaw    = state[4];
            }
            if (std::abs(v_long) > 1e-6 || std::abs(v_lat) > 1e-6) {
                beta = std::atan2(v_lat, v_long);
            }
            break;
    }

    const double speed = std::hypot(v_long, v_lat);
    t.speed            = speed;
    t.v_long           = v_long;
    t.v_lat            = v_lat;

    const double heading = yaw + beta;
    t.v_global_x        = speed * std::cos(heading);
    t.v_global_y        = speed * std::sin(heading);

    t.a_long = accel_output.acceleration;

    switch (sim.model) {
        case ModelType::MB:
            if (state.size() > 5) {
                t.a_lat = v_long * state[5];
            }
            break;
        default:
            if (wheelbase > 0.0) {
                t.a_lat = v_long * v_long * std::tan(delta) / wheelbase;
            } else {
                t.a_lat = 0.0;
            }
            break;
    }

    if (simulator) {
        t.speed             = std::max(0.0, simulator->speed());
        t.low_speed_engaged = simulator->safety().engaged();
    }

    if (accel_controller) {
        t.soc = accel_controller->powertrain().soc();
    }

    return t;
}

static double simulation_speed(const Simulation& sim)
{
    if (sim.integrator) {
        return std::max(0.0, sim.integrator->speed());
    }
    switch (sim.model) {
        case ModelType::MB:
            if (sim.x.size() > 10) {
                return std::hypot(sim.x[3], sim.x[10]);
            }
            break;
        default:
            if (sim.x.size() > 3) {
                return std::abs(sim.x[3]);
            }
            break;
    }
    return 0.0;
}

// --------------------------------------------------------------
// Drawing helpers
// --------------------------------------------------------------

struct Vec2 {
    double x;
    double y;
};

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

static Pose extract_pose(const Simulation& sim)
{
    Pose pose{};
    const auto& state = sim.integrator ? sim.integrator->state() : sim.x;
    if (state.size() >= 5) {
        pose.x   = state[0];
        pose.y   = state[1];
        pose.yaw = state[4];
    }
    return pose;
}

static void draw_car(SDL_Renderer* renderer,
                     const Simulation& sim,
                     int window_w, int window_h,
                     double pixels_per_meter)
{
    const Pose pose = extract_pose(sim);

    const double x_world = pose.x;
    const double y_world = pose.y;
    const double yaw     = pose.yaw;

    double L = sim.params.l;
    double W = sim.params.w;
    if (L <= 0.0) L = sim.params.a + sim.params.b;
    if (L <= 0.0) L = 4.0;
    if (W <= 0.0) W = sim.params.T_f;
    if (W <= 0.0) W = 1.8;

    const double halfL = 0.5 * L;
    const double halfW = 0.5 * W;

    Vec2 body_front{ halfL, 0.0 };
    Vec2 body_rl  { -halfL, -halfW };
    Vec2 body_rr  { -halfL,  halfW };

    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    auto body_to_world = [&](const Vec2& v) -> Vec2 {
        return {
            x_world + v.x * c - v.y * s,
            y_world + v.x * s + v.y * c
        };
    };

    Vec2 w_front = body_to_world(body_front);
    Vec2 w_rl    = body_to_world(body_rl);
    Vec2 w_rr    = body_to_world(body_rr);

    auto world_to_screen = [&](const Vec2& v) -> SDL_Point {
        SDL_Point p;
        p.x = static_cast<int>(window_w / 2 + v.x * pixels_per_meter);
        p.y = static_cast<int>(window_h / 2 - v.y * pixels_per_meter);
        return p;
    };

    SDL_Point p_front = world_to_screen(w_front);
    SDL_Point p_rl    = world_to_screen(w_rl);
    SDL_Point p_rr    = world_to_screen(w_rr);

    SDL_RenderDrawLine(renderer, p_front.x, p_front.y, p_rl.x, p_rl.y);
    SDL_RenderDrawLine(renderer, p_rl.x,    p_rl.y,    p_rr.x, p_rr.y);
    SDL_RenderDrawLine(renderer, p_rr.x,    p_rr.y,    p_front.x, p_front.y);
}

// --------------------------------------------------------------
// Simulation init using init_*
// --------------------------------------------------------------

static vsim::LowSpeedSafetyConfig reset_simulation(Simulation& sim,
                                                   ModelType model,
                                                   int vehicle_id,
                                                   const vio::ConfigManager& configs,
                                                   const std::string& param_root = {})
{
    std::optional<vio::ConfigManager> override_configs{};
    if (!param_root.empty()) {
        override_configs.emplace(std::filesystem::path{}, std::filesystem::path(param_root));
    }
    const auto& active_configs = override_configs ? *override_configs : configs;

    sim.model      = model;
    sim.vehicle_id = vehicle_id;
    sim.params     = active_configs.load_vehicle_parameters(vehicle_id);
    sim.u.assign(2, 0.0);
    sim.integrator.reset();

    // core init state: [sx, sy, delta, v, psi, dotPsi, beta]
    std::vector<double> core{
        0.0,  // sx
        0.0,  // sy
        0.0,  // delta
        0.0,  // v
        0.0,  // psi
        0.0,  // dotPsi
        0.0   // beta
    };

    // Pass the core state (before any init_* expansion) to the simulator so
    // that model-specific init functions are applied exactly once.
    sim.x0 = core;
    sim.x  = core;

    auto safety_cfg = active_configs.load_low_speed_safety_config(model);
    auto iface      = build_model_interface(model);
    auto safety     = build_low_speed_safety(model, sim.params, safety_cfg);
    sim.integrator = std::make_unique<vsim::VehicleSimulator>(
        std::move(iface),
        sim.params,
        static_cast<double>(sim.dt),
        std::move(safety));
    sim.integrator->reset(sim.x0);
    sim.x = sim.integrator->state();
    return safety_cfg;
}

// --------------------------------------------------------------
// Main
// --------------------------------------------------------------

int main(int, char**)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
        std::fprintf(stderr, "Error: SDL_Init() failed: %s\n", SDL_GetError());
        return 1;
    }

    const int window_w_init = 1280;
    const int window_h_init = 720;

    SDL_Window* window = SDL_CreateWindow(
        "CommonRoad Vehicle Demo",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        window_w_init, window_h_init,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_SHOWN
    );
    if (!window) {
        std::fprintf(stderr, "Error: SDL_CreateWindow() failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );
    if (!renderer) {
        std::fprintf(stderr, "Error: SDL_CreateRenderer() failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // ImGui init
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    auto shutdown_with_message = [&](const char* msg) -> int {
        if (msg) {
            std::fprintf(stderr, "Error: %s\n", msg);
        }
        ImGui_ImplSDLRenderer2_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    };

    // Simulation setup
    vio::ConfigManager config_manager{};
    std::optional<vio::ConfigManager> override_configs{};
    auto active_configs = [&]() -> const vio::ConfigManager& {
        if (override_configs) {
            return *override_configs;
        }
        return config_manager;
    };

    Simulation sim{};
    sim.dt = 0.01f;

    ModelType currentModel = ModelType::ST;
    int currentVehicleIdx  = 0; // 0..3 -> IDs 1..4
    const int vehicle_ids[4] = {1, 2, 3, 4};
    std::string param_root;
    enforce_model_dt_limits(sim, currentModel, active_configs());
    std::string dt_warning_message;
    double dt_warning_expires = 0.0;

    vsim::LowSpeedSafetyConfig low_speed_cfg{};
    try {
        low_speed_cfg = reset_simulation(
            sim,
            currentModel,
            vehicle_ids[currentVehicleIdx],
            active_configs(),
            param_root);
    } catch (const std::exception& e) {
        return shutdown_with_message(e.what());
    }

    vutils::SteeringConfig steering_config;
    try {
        steering_config = active_configs().load_steering_config();
    } catch (const std::exception& e) {
        return shutdown_with_message(e.what());
    }

    vutils::SteeringWheel         steering_wheel;
    vutils::FinalSteerController  steer_controller;
    vutils::SteeringWheel::Output wheel_state{};
    vutils::FinalSteerController::Output steer_state{};

    longi::PowertrainConfig           powertrain_cfg;
    longi::AeroConfig                 aero_cfg;
    longi::RollingResistanceConfig    rolling_cfg;
    longi::BrakeConfig                brake_cfg;
    longi::FinalAccelControllerConfig accel_cfg;

    try {
        powertrain_cfg = active_configs().load_powertrain_config();
        aero_cfg       = active_configs().load_aero_config();
        rolling_cfg    = active_configs().load_rolling_resistance_config();
        brake_cfg      = active_configs().load_brake_config();
        accel_cfg      = active_configs().load_final_accel_controller_config();
    } catch (const std::exception& e) {
        return shutdown_with_message(e.what());
    }

    accel_cfg.stop_speed_epsilon = low_speed_cfg.stop_speed_epsilon;

    auto build_accel_controller = [&]() {
        const double mass         = sim.params.m;
        const double wheel_radius = sim.params.R_w;
        if (!std::isfinite(mass) || mass <= 0.0) {
            throw std::runtime_error("Vehicle parameters must provide positive mass 'm'");
        }
        if (!std::isfinite(wheel_radius) || wheel_radius <= 0.0) {
            throw std::runtime_error("Vehicle parameters must provide positive wheel radius 'R_w'");
        }
        return std::make_unique<longi::FinalAccelController>(
            mass,
            wheel_radius,
            powertrain_cfg,
            aero_cfg,
            rolling_cfg,
            brake_cfg,
            accel_cfg);
    };

    std::unique_ptr<longi::FinalAccelController> accel_controller;
    try {
        accel_controller = build_accel_controller();
        accel_controller->reset();
    } catch (const std::exception& e) {
        return shutdown_with_message(e.what());
    }
    longi::ControllerOutput accel_output{};

    auto sync_controllers = [&]() {
        steering_wheel = vutils::SteeringWheel(steering_config.wheel, sim.params.steering);
        steer_controller =
            vutils::FinalSteerController(steering_config.final, sim.params.steering);

        const double current_angle = (sim.x.size() > 2) ? sim.x[2] : 0.0;
        steering_wheel.reset(current_angle);
        steer_controller.reset(current_angle);
        wheel_state = steering_wheel.last_output();
        steer_state = steer_controller.last_output();
        if (accel_controller) {
            accel_controller = build_accel_controller();
            accel_controller->reset();
        }
        accel_output = longi::ControllerOutput{};
    };

    try {
        sync_controllers();
    } catch (const std::exception& e) {
        return shutdown_with_message(e.what());
    }

    bool running = true;
    Uint64 prev_counter = SDL_GetPerformanceCounter();
    double sim_time = 0.0;
    float keyboard_brake_bias = 1.0f;

    while (running) {
        bool request_reset = false;
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                running = false;
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window))
                running = false;
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                } else if (event.key.keysym.sym == SDLK_r) {
                    request_reset = true;
                }
            }
        }

        if (request_reset) {
            if (sim.integrator) {
                sim.integrator->reset(sim.x0);
                sim.x = sim.integrator->state();
            }
            sim.u.assign(2, 0.0);
            try {
                sync_controllers();
            } catch (const std::exception& e) {
                return shutdown_with_message(e.what());
            }
            sim_time     = 0.0;
            prev_counter = SDL_GetPerformanceCounter();
            continue;
        }

        const Uint8* keys = SDL_GetKeyboardState(nullptr);

        double throttle_cmd = 0.0;
        double brake_cmd    = 0.0;
        if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP])    throttle_cmd = 1.0;
        if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN])
            brake_cmd = std::max(brake_cmd, static_cast<double>(keyboard_brake_bias));
        if (keys[SDL_SCANCODE_SPACE])                          brake_cmd = 1.0;
        throttle_cmd = std::clamp(throttle_cmd, 0.0, 1.0);
        brake_cmd    = std::clamp(brake_cmd, 0.0, 1.0);
        if (brake_cmd > 0.0) {
            throttle_cmd = 0.0;
        }

        double steer_nudge = 0.0;
        if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT])  steer_nudge += 1.0;
        if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) steer_nudge -= 1.0;
        if (steer_nudge > 1.0) steer_nudge = 1.0;
        if (steer_nudge < -1.0) steer_nudge = -1.0;

        wheel_state = steering_wheel.update(steer_nudge, sim.dt);
        const double current_delta = (sim.x.size() > 2) ? sim.x[2] : wheel_state.angle;
        steer_state = steer_controller.update(wheel_state.angle, current_delta, sim.dt);
        sim.u[0] = steer_state.rate;

        const double speed = std::max(0.0, simulation_speed(sim));
        if (accel_controller) {
            longi::DriverIntent intent{};
            intent.throttle = throttle_cmd;
            intent.brake    = brake_cmd;
            accel_output    = accel_controller->step(intent, speed, sim.dt);
            sim.u[1]        = std::clamp(accel_output.acceleration,
                                         accel_cfg.accel_min,
                                         accel_cfg.accel_max);
        } else {
            const double accel_raw = accel_cfg.accel_max * throttle_cmd + accel_cfg.accel_min * (-brake_cmd);
            const double accel     = std::clamp(accel_raw, accel_cfg.accel_min, accel_cfg.accel_max);
            accel_output          = longi::ControllerOutput{};
            accel_output.throttle = throttle_cmd;
            accel_output.brake    = brake_cmd;
            accel_output.acceleration = accel;
            sim.u[1] = accel;
        }

        if (sim.integrator) {
            sim.integrator->set_dt(static_cast<double>(sim.dt));
            const auto& updated_state = sim.integrator->step(sim.u);
            sim.x = updated_state;
            sim_time += sim.dt;
        }

        // ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        const double now_seconds = static_cast<double>(SDL_GetTicks64()) * 0.001;

        {
            ImGui::Begin("Simulation Control");

            ImGui::Text("Time: %.2f s", sim_time);
            ImGui::Separator();

            ImGui::Text("Keyboard controls");
            ImGui::Text("W/Up: throttle  A/D or ←/→: steer  R: reset  ESC: quit");
            if (ImGui::SliderFloat("Keyboard brake bias", &keyboard_brake_bias, 0.0f, 1.0f, "%.2f")) {
                keyboard_brake_bias = std::clamp(keyboard_brake_bias, 0.0f, 1.0f);
            }
            ImGui::TextWrapped(
                "S/Down applies %.0f%%%% braking using the bias slider for proportional stops."
                " Hold SPACE to request 100%%%% emergency braking.",
                keyboard_brake_bias * 100.0f);
            ImGui::Separator();

            int model_idx = static_cast<int>(currentModel);
            const char* model_items[] = {
                "KS (vehicle_dynamics_ks)",
                "KS CoG (vehicle_dynamics_ks_cog)",
                "KST trailer (vehicle_dynamics_kst)",
                "Multi-body (vehicle_dynamics_mb)",
                "ST (vehicle_dynamics_st)",
                "STD (vehicle_dynamics_std)"
            };
            if (ImGui::Combo("Model", &model_idx, model_items, IM_ARRAYSIZE(model_items))) {
                currentModel = static_cast<ModelType>(model_idx);
                const auto timing = active_configs().load_model_timing(currentModel);
                const float requested_dt = sim.dt;
                const bool dt_was_above_max = requested_dt > timing.max_dt + 1e-6f;
                const bool dt_adjusted = enforce_model_dt_limits(sim, currentModel, active_configs());
                if (dt_adjusted && dt_was_above_max) {
                    set_dt_warning(dt_warning_message,
                                   dt_warning_expires,
                                   now_seconds,
                                   currentModel,
                                   requested_dt,
                                   timing.max_dt);
                }
                try {
                    low_speed_cfg = reset_simulation(
                        sim,
                        currentModel,
                        vehicle_ids[currentVehicleIdx],
                        active_configs(),
                        param_root);
                } catch (const std::exception& e) {
                    return shutdown_with_message(e.what());
                }
                accel_cfg.stop_speed_epsilon = low_speed_cfg.stop_speed_epsilon;
                try {
                    sync_controllers();
                } catch (const std::exception& e) {
                    return shutdown_with_message(e.what());
                }
                sim_time     = 0.0;
                prev_counter = SDL_GetPerformanceCounter();
            }

            const char* vehicle_items[] = {
                "Vehicle 1 (Ford Escort)",
                "Vehicle 2 (BMW 320i)",
                "Vehicle 3 (VW Vanagon)",
                "Vehicle 4 (Semi-trailer truck)"
            };
            if (ImGui::Combo("Vehicle", &currentVehicleIdx,
                             vehicle_items, IM_ARRAYSIZE(vehicle_items))) {
                try {
                    low_speed_cfg = reset_simulation(
                        sim,
                        currentModel,
                        vehicle_ids[currentVehicleIdx],
                        active_configs(),
                        param_root);
                } catch (const std::exception& e) {
                    return shutdown_with_message(e.what());
                }
                accel_cfg.stop_speed_epsilon = low_speed_cfg.stop_speed_epsilon;
                try {
                    sync_controllers();
                } catch (const std::exception& e) {
                    return shutdown_with_message(e.what());
                }
                sim_time     = 0.0;
                prev_counter = SDL_GetPerformanceCounter();
            }

            const auto timing = active_configs().load_model_timing(currentModel);
            ImGui::Text("Nominal dt: %.3f s (max %.3f s)", timing.nominal_dt, timing.max_dt);
            float slider_dt = sim.dt;
            if (ImGui::SliderFloat("dt [s]", &slider_dt, kSliderMinDt, kSliderMaxDt, "%.3f")) {
                const float requested_dt = slider_dt;
                bool exceeded_max        = false;
                if (slider_dt > timing.max_dt) {
                    slider_dt    = timing.max_dt;
                    exceeded_max = true;
                }
                if (slider_dt < kSliderMinDt) {
                    slider_dt = kSliderMinDt;
                }
                sim.dt = slider_dt;
                reconfigure_simulator_dt(sim, low_speed_cfg);
                if (exceeded_max) {
                    set_dt_warning(dt_warning_message,
                                   dt_warning_expires,
                                   now_seconds,
                                   currentModel,
                                   requested_dt,
                                   timing.max_dt);
                }
            }
            if (dt_warning_expires > now_seconds) {
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.2f, 1.0f), "%s", dt_warning_message.c_str());
            }

            ImGui::Separator();
            ImGui::Text("Inputs:");
            ImGui::Text("Wheel angle:  %+6.3f rad", wheel_state.angle);
            ImGui::Text("Wheel rate:   %+6.3f rad/s", wheel_state.rate);
            ImGui::Text("Steer target: %+6.3f rad", steer_state.filtered_target);
            ImGui::Text("Steer angle (cmd): %+6.3f rad", steer_state.angle);
            const bool has_sim_delta = sim.x.size() > 2;
            const double sim_delta   = has_sim_delta ? sim.x[2] : 0.0;
            if (has_sim_delta) {
                ImGui::Text("Steer angle (sim): %+6.3f rad", sim_delta);
            } else {
                ImGui::Text("Steer angle (sim):    N/A");
            }
            ImGui::Text("Steer rate:   %+6.3f rad/s", steer_state.rate);
            ImGui::Text("Accel:        %+6.3f m/s^2", sim.u[1]);
            ImGui::Text("Throttle cmd:  %4.2f", accel_output.throttle);
            ImGui::Text("Brake cmd:     %4.2f", accel_output.brake);
            ImGui::Text("Drive force:   %+6.3f N", accel_output.drive_force);
            ImGui::Text("Brake force:   %+6.3f N", accel_output.brake_force);
            ImGui::Text("Regen force:   %+6.3f N", accel_output.regen_force);

            ImGui::Separator();
            ImGui::Text("State:");
            if (sim.x.size() >= 5) {
                ImGui::Text("x:   %8.3f m",   sim.x[0]);
                ImGui::Text("y:   %8.3f m",   sim.x[1]);
                ImGui::Text("psi: %8.3f rad", sim.x[4]);
                ImGui::Text("v:   %8.3f m/s", sim.x[3]);
            } else {
                ImGui::Text("State size: %zu", sim.x.size());
            }

            Telemetry tel = compute_telemetry(sim, accel_output, accel_controller.get());
            ImGui::Separator();
            ImGui::Text("Telemetry:");
            ImGui::Text("Speed:     %8.3f m/s",  tel.speed);
            ImGui::Text("v_long:    %8.3f m/s",  tel.v_long);
            ImGui::Text("v_lat:     %8.3f m/s",  tel.v_lat);
            ImGui::Text("v_global.x %8.3f m/s",  tel.v_global_x);
            ImGui::Text("v_global.y %8.3f m/s",  tel.v_global_y);
            ImGui::Text("a_long:    %8.3f m/s^2", tel.a_long);
            ImGui::Text("a_lat:     %8.3f m/s^2", tel.a_lat);
            ImGui::Text("Low-speed safety: %s", tel.low_speed_engaged ? "ENGAGED" : "Released");
            if (accel_controller) {
                ImGui::Text("Battery SOC:  %6.2f %%", tel.soc * 100.0);
            } else {
                ImGui::Text("Battery SOC:      N/A");
            }

            ImGui::End();
        }

        ImGui::Render();

        int win_w, win_h;
        SDL_GetWindowSize(window, &win_w, &win_h);

        SDL_SetRenderDrawColor(renderer, 20, 20, 25, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 60, 60, 80, 255);
        {
            const int cx = win_w / 2;
            const int cy = win_h / 2;
            SDL_RenderDrawLine(renderer, 0,  cy, win_w, cy);
            SDL_RenderDrawLine(renderer, cx, 0, cx, win_h);
        }

        SDL_SetRenderDrawColor(renderer, 200, 200, 50, 255);
        const double pixels_per_meter = 10.0;
        draw_car(renderer, sim, win_w, win_h, pixels_per_meter);

        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);

        Uint64 curr_counter = SDL_GetPerformanceCounter();
        Uint64 freq         = SDL_GetPerformanceFrequency();
        double frame_dt     = static_cast<double>(curr_counter - prev_counter) /
                              static_cast<double>(freq);
        prev_counter = curr_counter;
        (void)frame_dt;
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
