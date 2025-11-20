// app/main.cpp
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

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

#include "common/errors.hpp"
#include "io/config_manager.hpp"
#include "simulation/model_timing.hpp"
#include "simulation/simulation_daemon.hpp"
#include "telemetry/telemetry_imgui.hpp"

namespace vsim  = velox::simulation;
namespace vio   = velox::io;
namespace vtel  = velox::telemetry;

namespace {

constexpr float kSliderMinDt   = vsim::kMinStableDt;
constexpr float kSliderMaxDt   = 0.05f;
const std::array<int, 4> kVehicleIds{1, 2, 3, 4};

struct SimulationUiState {
    vsim::ModelType model{vsim::ModelType::MB};
    int             vehicle_index{0};
    double          requested_dt{vsim::model_timing(vsim::ModelType::MB).nominal_dt};
    std::string     dt_warning{};
    double          dt_warning_expires{0.0};
};

vsim::ModelTiming::StepSchedule plan_schedule(const vsim::ModelTimingInfo& info, double requested_dt)
{
    vsim::ModelTiming timing{info};
    return timing.plan_steps(requested_dt);
}

void note_timing_message(const vsim::ModelTiming::StepSchedule& schedule,
                         const vsim::ModelTimingInfo& info,
                         double now_seconds,
                         std::string& message,
                         double& expires_at)
{
    if (schedule.clamped_to_min) {
        char buffer[128];
        std::snprintf(buffer,
                      sizeof(buffer),
                      "Requested dt below stability limit; clamped to %.3f s",
                      static_cast<float>(schedule.clamped_dt));
        message    = buffer;
        expires_at = now_seconds + 4.0;
    } else if (schedule.used_substeps) {
        const auto max_substep = *std::max_element(schedule.substeps.begin(), schedule.substeps.end());
        char buffer[192];
        std::snprintf(buffer,
                      sizeof(buffer),
                      "Requested dt exceeds %.3f s; splitting into %zu sub-steps (max %.3f s)",
                      info.max_dt,
                      schedule.substeps.size(),
                      static_cast<float>(max_substep));
        message    = buffer;
        expires_at = now_seconds + 4.0;
    }
}

void draw_car(SDL_Renderer* renderer,
              const vsim::SimulationDaemon& daemon,
              const vtel::SimulationTelemetry& telemetry,
              int window_w,
              int window_h,
              double pixels_per_meter)
{
    const auto& params = daemon.vehicle_parameters();
    const double x_world = telemetry.pose.x;
    const double y_world = telemetry.pose.y;
    const double yaw     = telemetry.pose.yaw;

    double L = params.l;
    double W = params.w;
    if (L <= 0.0) L = params.a + params.b;
    if (L <= 0.0) L = 4.0;
    if (W <= 0.0) W = params.T_f;
    if (W <= 0.0) W = 1.8;

    const double halfL = 0.5 * L;
    const double halfW = 0.5 * W;

    struct Vec2 { double x; double y; };
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

} // namespace

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

    vio::ConfigManager config_manager{};
    SimulationUiState  ui{};
    vsim::ModelTimingInfo timing_info = config_manager.load_model_timing(ui.model);
    ui.requested_dt = timing_info.nominal_dt;

    vsim::SimulationDaemon daemon({
        .model = ui.model,
        .vehicle_id = kVehicleIds[ui.vehicle_index],
        .config_root = config_manager.config_root(),
        .parameter_root = config_manager.parameter_root(),
        .log_sink = {}
    });

    vtel::SimulationTelemetry telemetry = daemon.telemetry();

    bool  running            = true;
    float keyboard_brake_bias = 1.0f;
    vsim::UserInput current_input{};
    std::string input_error_message;
    double input_error_expires = 0.0;

    Uint64 prev_counter = SDL_GetPerformanceCounter();

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

        const double now_seconds = static_cast<double>(SDL_GetTicks64()) * 0.001;
        const auto   schedule    = plan_schedule(
            timing_info,
            std::max(ui.requested_dt, static_cast<double>(vsim::kMinStableDt)));
        note_timing_message(schedule, timing_info, now_seconds, ui.dt_warning, ui.dt_warning_expires);

        if (request_reset) {
            try {
                vsim::ResetParams params{};
                params.model      = ui.model;
                params.vehicle_id = kVehicleIds[ui.vehicle_index];
                params.dt         = schedule.clamped_dt;
                daemon.reset(params);
                telemetry = daemon.telemetry();
                current_input.timestamp = 0.0;
            } catch (const std::exception& e) {
                return shutdown_with_message(e.what());
            }
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
        steer_nudge = std::clamp(steer_nudge, -1.0, 1.0);

        current_input.timestamp             = telemetry.totals.simulation_time_s;
        current_input.dt                    = schedule.clamped_dt;
        current_input.steering_nudge        = steer_nudge;
        current_input.longitudinal.throttle = throttle_cmd;
        current_input.longitudinal.brake    = brake_cmd;

        bool input_valid = true;
        vsim::UserInput sanitized_input{};
        try {
            sanitized_input = current_input.clamped();
        } catch (const velox::errors::VeloxError& e) {
            input_valid = false;
            input_error_message = e.what();
            input_error_expires = now_seconds + 4.0;
        }

        if (input_valid) {
            try {
                telemetry = daemon.step(sanitized_input);
            } catch (const velox::errors::VeloxError& e) {
                return shutdown_with_message(e.what());
            }
        }

        // ImGui frame
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        {
            ImGui::Begin("Simulation Control");

            ImGui::Text("Time: %.2f s", telemetry.totals.simulation_time_s);
            const bool drift_on           = telemetry.traction.drift_mode;
            const bool low_speed_latched  = telemetry.low_speed_engaged;
            const bool detector_forced    = telemetry.detector_forced;
            const char* stage_label       = vtel::safety_stage_to_string(telemetry.safety_stage);
            const ImVec4 drift_color      = drift_on ? ImVec4(0.78f, 0.92f, 0.36f, 1.0f)
                                                     : ImVec4(0.55f, 0.55f, 0.55f, 1.0f);
            const ImVec4 latch_color      = low_speed_latched ? ImVec4(1.0f, 0.62f, 0.26f, 1.0f)
                                                             : ImVec4(0.55f, 0.55f, 0.55f, 1.0f);
            const ImVec4 detector_color   = detector_forced ? ImVec4(1.0f, 0.35f, 0.35f, 1.0f) : latch_color;
            ImGui::TextColored(drift_color, "Drift mode: %s", drift_on ? "ACTIVE" : "Off");
            ImGui::TextColored(latch_color,
                               "Low-speed safety latch: %s",
                               low_speed_latched ? "ENGAGED" : "Released");
            ImGui::TextColored(detector_color,
                               "Safety stage: %s (severity %.2f)%s",
                               stage_label,
                               telemetry.detector_severity,
                               detector_forced ? " [detector]" : "");
            ImGui::Separator();

            ImGui::Text("Keyboard controls");
            ImGui::Text("W/Up: throttle  A/D or \u2190/\u2192: steer  R: reset  ESC: quit");
            if (ImGui::SliderFloat("Keyboard brake bias", &keyboard_brake_bias, 0.0f, 1.0f, "%.2f")) {
                keyboard_brake_bias = std::clamp(keyboard_brake_bias, 0.0f, 1.0f);
            }
            ImGui::TextWrapped(
                "S/Down applies %.0f%%%% braking using the bias slider for proportional stops."
                " Hold SPACE to request 100%%%% emergency braking.",
                keyboard_brake_bias * 100.0f);
            if (input_error_expires > now_seconds) {
                ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f),
                                   "Input error: %s",
                                   input_error_message.c_str());
            }
            ImGui::Separator();

            int model_idx = static_cast<int>(ui.model);
            const char* model_items[] = {
                vsim::model_display_name(vsim::ModelType::MB),
                vsim::model_display_name(vsim::ModelType::ST),
                vsim::model_display_name(vsim::ModelType::STD)
            };
            if (ImGui::Combo("Model", &model_idx, model_items, IM_ARRAYSIZE(model_items))) {
                ui.model     = static_cast<vsim::ModelType>(model_idx);
                timing_info  = config_manager.load_model_timing(ui.model);
                ui.requested_dt = timing_info.nominal_dt;
                try {
                    vsim::ResetParams params{};
                    params.model      = ui.model;
                    params.vehicle_id = kVehicleIds[ui.vehicle_index];
                    params.dt         = ui.requested_dt;
                    daemon.reset(params);
                    telemetry = daemon.telemetry();
                    ui.dt_warning.clear();
                } catch (const std::exception& e) {
                    return shutdown_with_message(e.what());
                }
            }

            const char* vehicle_items[] = {
                "Vehicle 1 (Ford Escort)",
                "Vehicle 2 (BMW 320i)",
                "Vehicle 3 (VW Vanagon)",
                "Vehicle 4 (Semi-trailer truck)"
            };
            if (ImGui::Combo("Vehicle", &ui.vehicle_index, vehicle_items, IM_ARRAYSIZE(vehicle_items))) {
                try {
                    vsim::ResetParams params{};
                    params.model      = ui.model;
                    params.vehicle_id = kVehicleIds[ui.vehicle_index];
                    params.dt         = ui.requested_dt;
                    daemon.reset(params);
                    telemetry = daemon.telemetry();
                    ui.dt_warning.clear();
                } catch (const std::exception& e) {
                    return shutdown_with_message(e.what());
                }
            }

            ImGui::Text("Nominal dt: %.3f s (max %.3f s)", timing_info.nominal_dt, timing_info.max_dt);
            float slider_dt = static_cast<float>(ui.requested_dt);
            if (ImGui::SliderFloat("dt [s]", &slider_dt, kSliderMinDt, kSliderMaxDt, "%.3f")) {
                ui.requested_dt = slider_dt;
                note_timing_message(schedule, timing_info, now_seconds, ui.dt_warning, ui.dt_warning_expires);
            }
            if (ui.dt_warning_expires > now_seconds) {
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.2f, 1.0f), "%s", ui.dt_warning.c_str());
            }
            ImGui::Text("Current schedule: %zu sub-steps (%.3f s total)",
                        schedule.substeps.size(),
                        schedule.total_duration());

            ImGui::Separator();
            ImGui::Text("Inputs:");
            ImGui::Text("Steer nudge: %+6.3f", current_input.steering_nudge);
            ImGui::Text("Accel request: %+6.3f m/s^2", current_input.longitudinal.throttle - current_input.longitudinal.brake);
            ImGui::Text("Throttle cmd:  %4.2f", current_input.longitudinal.throttle);
            ImGui::Text("Brake cmd:     %4.2f", current_input.longitudinal.brake);

            ImGui::Separator();
            ImGui::Text("State:");
            ImGui::Text("x:   %8.3f m",   telemetry.pose.x);
            ImGui::Text("y:   %8.3f m",   telemetry.pose.y);
            ImGui::Text("psi: %8.3f rad", telemetry.pose.yaw);
            ImGui::Text("v:   %8.3f m/s", telemetry.velocity.speed);
            ImGui::Text("Distance: %.2f m", telemetry.totals.distance_traveled_m);
            ImGui::Text("Energy:   %.1f J", telemetry.totals.energy_consumed_joules);
            ImGui::Text("Low-speed safety: %s (stage: %s%s)",
                        telemetry.low_speed_engaged ? "engaged" : "off",
                        vtel::safety_stage_to_string(telemetry.safety_stage),
                        telemetry.detector_forced ? ", detector" : "");

            vtel::draw_telemetry_imgui(telemetry);

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
        draw_car(renderer, daemon, telemetry, win_w, win_h, pixels_per_meter);

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
