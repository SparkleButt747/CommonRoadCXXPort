// app/main.cpp
#include <cmath>
#include <cstdio>
#include <exception>
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

#include "vehiclemodels/vehicle_dynamics_ks.hpp"
#include "vehiclemodels/vehicle_dynamics_st.hpp"
#include "vehiclemodels/vehicle_dynamics_std.hpp"
#include "utils/vehicle_dynamics_ks_cog.hpp"
#include "utils/steering_controller.hpp"

#include "vehiclemodels/init_ks.hpp"
#include "vehiclemodels/init_st.hpp"
#include "vehiclemodels/init_std.hpp"

namespace vm = vehiclemodels;
namespace vutils = vehiclemodels::utils;

// --------------------------------------------------------------
// Simulation model dispatch
// --------------------------------------------------------------

enum class ModelType {
    KS_REAR = 0,
    KS_COG  = 1,
    ST      = 2,
    STD     = 3
};

struct Simulation {
    ModelType             model;
    int                   vehicle_id;
    vm::VehicleParameters params;
    std::vector<double>   x;     // state
    std::vector<double>   u;     // input [steer_rate, accel]
    float                 dt;    // integration step [s]
};

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

// compute RHS for current model
static std::vector<double> compute_rhs(const Simulation& sim)
{
    switch (sim.model) {
        case ModelType::KS_REAR:
            return vm::vehicle_dynamics_ks(sim.x, sim.u, sim.params);

        case ModelType::KS_COG: {
            std::array<double, 5> x_arr{};
            std::array<double, 2> u_arr{};
            for (int i = 0; i < 5; ++i) x_arr[i] = sim.x[i];
            u_arr[0] = sim.u[0];
            u_arr[1] = sim.u[1];
            auto f_arr = vm::utils::vehicle_dynamics_ks_cog(x_arr, u_arr, sim.params);
            std::vector<double> f(5);
            for (int i = 0; i < 5; ++i) f[i] = f_arr[i];
            return f;
        }

        case ModelType::ST:
            return vm::vehicle_dynamics_st(sim.x, sim.u, sim.params);

        case ModelType::STD:
            return vm::vehicle_dynamics_std(sim.x, sim.u, sim.params);
    }
    return {};
}

// --------------------------------------------------------------
// Telemetry
// --------------------------------------------------------------

struct Telemetry {
    double speed;
    double v_long;
    double v_lat;
    double v_global_x;
    double v_global_y;
    double a_long;
    double a_lat;
};

static Telemetry compute_telemetry(const Simulation& sim)
{
    Telemetry t{};

    const double L     = sim.params.a + sim.params.b;
    const double delta = (sim.x.size() > 2) ? sim.x[2] : 0.0;

    double v    = 0.0;
    double beta = 0.0;
    double yaw  = 0.0;

    switch (sim.model) {
        case ModelType::KS_REAR:
        case ModelType::KS_COG:
            if (sim.x.size() >= 5) {
                v   = sim.x[3];
                yaw = sim.x[4];
            }
            beta = 0.0;
            break;

        case ModelType::ST:
            if (sim.x.size() >= 7) {
                v    = sim.x[3];
                yaw  = sim.x[4];
                beta = sim.x[6];
            }
            break;

        case ModelType::STD:
            if (sim.x.size() >= 9) {
                v    = sim.x[3];
                yaw  = sim.x[4];
                beta = sim.x[6];
            }
            break;
    }

    t.speed  = v;
    t.v_long = v * std::cos(beta);
    t.v_lat  = v * std::sin(beta);

    const double heading = yaw + beta;
    t.v_global_x = v * std::cos(heading);
    t.v_global_y = v * std::sin(heading);

    t.a_long = (sim.u.size() > 1) ? sim.u[1] : 0.0;
    if (L > 0.0) {
        t.a_lat = v * v * std::tan(delta) / L;
    } else {
        t.a_lat = 0.0;
    }

    return t;
}

// --------------------------------------------------------------
// Drawing helpers
// --------------------------------------------------------------

struct Vec2 {
    double x;
    double y;
};

static void draw_car(SDL_Renderer* renderer,
                     const Simulation& sim,
                     int window_w, int window_h,
                     double pixels_per_meter)
{
    if (sim.x.size() < 5) return;

    const double x_world = sim.x[0];
    const double y_world = sim.x[1];
    const double yaw     = sim.x[4];

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

static void reset_simulation(Simulation& sim,
                             ModelType model,
                             int vehicle_id,
                             const std::string& param_root = {})
{
    sim.model      = model;
    sim.vehicle_id = vehicle_id;
    sim.params     = load_vehicle_params(vehicle_id, param_root);
    sim.u.assign(2, 0.0);

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

    switch (model) {
        case ModelType::KS_REAR:
        case ModelType::KS_COG:
            sim.x = vm::init_ks(core);          // length 5
            break;
        case ModelType::ST:
            sim.x = vm::init_st(core);          // length 7
            break;
        case ModelType::STD:
            sim.x = vm::init_std(core, sim.params); // length 9
            break;
    }
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

    // Simulation setup
    Simulation sim{};
    sim.dt = 0.01f;

    ModelType currentModel = ModelType::ST;
    int currentVehicleIdx  = 0; // 0..3 -> IDs 1..4
    const int vehicle_ids[4] = {1, 2, 3, 4};
    std::string param_root;

    reset_simulation(sim, currentModel, vehicle_ids[currentVehicleIdx], param_root);

    vutils::SteeringConfig steering_config;
    try {
        steering_config = vutils::SteeringConfig::load_default();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    vutils::SteeringWheel         steering_wheel;
    vutils::FinalSteerController  steer_controller;
    vutils::SteeringWheel::Output wheel_state{};
    vutils::FinalSteerController::Output steer_state{};

    auto sync_controllers = [&]() {
        steering_wheel = vutils::SteeringWheel(steering_config.wheel, sim.params.steering);
        steer_controller =
            vutils::FinalSteerController(steering_config.final, sim.params.steering);

        const double current_angle = (sim.x.size() > 2) ? sim.x[2] : 0.0;
        steering_wheel.reset(current_angle);
        steer_controller.reset(current_angle);
        wheel_state = steering_wheel.last_output();
        steer_state = steer_controller.last_output();
    };

    sync_controllers();

    bool running = true;
    Uint64 prev_counter = SDL_GetPerformanceCounter();
    double sim_time = 0.0;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                running = false;
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window))
                running = false;
            if (event.type == SDL_KEYDOWN &&
                event.key.keysym.sym == SDLK_ESCAPE)
                running = false;
        }

        const Uint8* keys = SDL_GetKeyboardState(nullptr);

        double throttle_cmd = 0.0;
        if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP])    throttle_cmd += 1.0;
        if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN])  throttle_cmd -= 1.0;

        double steer_nudge = 0.0;
        if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT])  steer_nudge += 1.0;
        if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) steer_nudge -= 1.0;
        if (steer_nudge > 1.0) steer_nudge = 1.0;
        if (steer_nudge < -1.0) steer_nudge = -1.0;

        const double MAX_ACCEL = 4.0;

        wheel_state = steering_wheel.update(steer_nudge, sim.dt);
        const double current_delta = (sim.x.size() > 2) ? sim.x[2] : wheel_state.angle;
        steer_state = steer_controller.update(wheel_state.angle, current_delta, sim.dt);

        sim.u[1] = throttle_cmd * MAX_ACCEL;
        sim.u[0] = steer_state.rate;

        // integrate one step
        auto f = compute_rhs(sim);
        if (f.size() == sim.x.size()) {
            for (std::size_t i = 0; i < sim.x.size(); ++i) {
                sim.x[i] += f[i] * sim.dt;
            }
            sim_time += sim.dt;
        }

        // ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        {
            ImGui::Begin("Simulation Control");

            ImGui::Text("Time: %.2f s", sim_time);
            ImGui::Separator();

            int model_idx = static_cast<int>(currentModel);
            const char* model_items[] = {
                "KS rear (vehicle_dynamics_ks)",
                "KS CoG (vehicle_dynamics_ks_cog)",
                "ST (vehicle_dynamics_st)",
                "STD (vehicle_dynamics_std)"
            };
            if (ImGui::Combo("Model", &model_idx, model_items, IM_ARRAYSIZE(model_items))) {
                currentModel = static_cast<ModelType>(model_idx);
                reset_simulation(sim, currentModel, vehicle_ids[currentVehicleIdx], param_root);
                sync_controllers();
            }

            const char* vehicle_items[] = {
                "Vehicle 1 (Ford Escort)",
                "Vehicle 2 (BMW 320i)",
                "Vehicle 3 (VW Vanagon)",
                "Vehicle 4 (Semi-trailer truck)"
            };
            if (ImGui::Combo("Vehicle", &currentVehicleIdx,
                             vehicle_items, IM_ARRAYSIZE(vehicle_items))) {
                reset_simulation(sim, currentModel, vehicle_ids[currentVehicleIdx], param_root);
                sync_controllers();
            }

            ImGui::SliderFloat("dt [s]", &sim.dt, 0.001f, 0.05f, "%.3f");

            ImGui::Separator();
            ImGui::Text("Inputs:");
            ImGui::Text("Wheel angle:  %+6.3f rad", wheel_state.angle);
            ImGui::Text("Wheel rate:   %+6.3f rad/s", wheel_state.rate);
            ImGui::Text("Steer target: %+6.3f rad", steer_state.filtered_target);
            ImGui::Text("Steer angle (cmd): %+6.3f rad", steer_state.angle);
            const double sim_delta = (sim.x.size() > 2) ? sim.x[2] : 0.0;
            ImGui::Text("Steer angle (sim): %+6.3f rad", sim_delta);
            ImGui::Text("Steer rate:   %+6.3f rad/s", steer_state.rate);
            ImGui::Text("Accel:        %+6.3f m/s^2", sim.u[1]);

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

            Telemetry tel = compute_telemetry(sim);
            ImGui::Separator();
            ImGui::Text("Telemetry:");
            ImGui::Text("Speed:     %8.3f m/s",  tel.speed);
            ImGui::Text("v_long:    %8.3f m/s",  tel.v_long);
            ImGui::Text("v_lat:     %8.3f m/s",  tel.v_lat);
            ImGui::Text("v_global.x %8.3f m/s",  tel.v_global_x);
            ImGui::Text("v_global.y %8.3f m/s",  tel.v_global_y);
            ImGui::Text("a_long:    %8.3f m/s^2", tel.a_long);
            ImGui::Text("a_lat:     %8.3f m/s^2", tel.a_lat);

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
