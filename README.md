# CommonRoadCXXPort

A lightweight C++17/20 port of the CommonRoad vehicle models with a small SDL/ImGui viewer. The repository now exposes the simulation runtime as a reusable static library (`velox`) with a simple daemon-style API for embedding in other applications.

## Package layout

- `CMakeLists.txt` – builds the `velox` static library, optional examples, and the SDL/ImGui demo app.
- `lib/` – headers and implementation of the public API (controllers, models, simulation, telemetry, logging, and error types).
- `parameters/` – vehicle parameter YAML files shipped with the library.
- `config/` – controller and timing configuration YAML files.
- `app/` – SDL + ImGui demo that consumes the daemon library exclusively.
- `examples/` – minimal integration samples built with `velox`.

## Public API overview

### Configuration and parameters
- `velox::io::ConfigManager` loads controller and timing YAML files and resolves parameter roots. It defaults to `parameters/` with `config/` as the sibling configuration directory.
- `velox::models::VehicleParameters` holds physical properties (mass, wheelbase, tire data, steering limits, etc.) and is populated by the configuration manager.

### Simulation daemon
- `velox::simulation::SimulationDaemon` owns the model interface, controllers, safety system, and a `VehicleSimulator`. Construct it with `SimulationDaemon::InitParams` to select the model, vehicle id, configuration root, and parameter root.
- `reset(const ResetParams&)` swaps models/vehicles, seeds the simulator, and reinitializes controllers and safety systems while preserving the configured roots.
- `step(const UserInput&)` advances the simulation by one request (internally sub-stepping as required by timing constraints) and returns a `telemetry::SimulationTelemetry` snapshot.
- `snapshot()` returns the last telemetry, current state vector, timestep, and accumulated simulated time without mutating the simulation.
- `telemetry()` exposes the last computed telemetry payload for callers that are not stepping every frame (e.g., render-only threads).

### User input and timing
- `UserInput` encodes throttle/brake, steering nudge, drift toggles, requested timestep `dt`, and the caller’s timestamp. The helper `clamped()` validates input and constrains it using `UserInputLimits` (defaults are exported as `kDefaultUserInputLimits`).
- Transmission/gear selection is intentionally kept out of the daemon inputs; if an ICE powertrain is added later, model it via a dedicated powertrain control block rather than exposing gearbox state directly.
- `ModelTiming` uses per-model `ModelTimingInfo` (`nominal_dt`, `max_dt`) to sub-divide large requested steps and clamp unstable `dt` requests to a safe minimum (`kMinStableDt`). `SimulationDaemon` applies the same scheduling internally, so hosts should pass their requested `dt` and optionally surface any clamping/sub-stepping to users.

### Telemetry schema
The daemon reports a `telemetry::SimulationTelemetry` struct on every step:
- `pose` – world-frame `x`, `y`, and `yaw`.
- `velocity` – scalar speed plus longitudinal, lateral, yaw-rate, and global-frame components.
- `acceleration` – longitudinal and lateral accelerations.
- `traction` – slip angle metrics, lateral force saturation, and the drift-mode flag reported by the safety system.
- `steering` – desired/actual angles and rates after steering filtering.
- `controller` – commanded acceleration, throttle/brake blending, and longitudinal force breakdown.
- `powertrain` – drive/regen torque, mechanical vs. battery power, and SOC estimates.
- `front_axle`/`rear_axle` – axle and wheel torques, slip, and friction utilization.
- `totals` – cumulative distance traveled, energy consumption (joules), and simulated time.
- `low_speed_engaged` – safety controller flag indicating clamped behavior at near-zero speed.

Use `telemetry::to_json` to serialize the payload or `telemetry::draw_telemetry_imgui` for debug UI rendering.

### Drift mode and low-speed safety
- The low-speed safety controller clamps yaw rate, slip angle, wheel speeds, and steering to keep models stable at near-zero speed. It exposes a relaxed “drift” profile with higher limits and looser clamping when controlled oversteer is desired.
- Each model’s default profile is defined in `config/low_speed_safety*.yaml`. The single-track drift model ships with drift mode enabled by default, while other models default to the normal profile.
- Enable or disable drift mode via `SimulationDaemon::InitParams::drift_enabled`, `ResetParams::drift_enabled`, or a `UserInput::drift_toggle` value ≥ `0.5`. Calling `SimulationDaemon::reset` without a drift override restores the model’s configured default and reinitializes controller integrators and safety latches.
- Run `drift_mode_demo` after building to see drift mode expand yaw/slip telemetry compared to the normal profile in a headless loop; it uses the single-track drift (`ModelType::STD`) model and flips the drift profile on/off between passes. The sample now also logs detector severity and safety stage during a brake-and-steer maneuver to demonstrate the emergency engagement path and provide a repeatable CI check.
- See `docs/safety_pipeline.md` for a deeper explanation of the staged safety pipeline, detector thresholds, drift defaults, and tuning guidance for new models.

### Error handling and logging
- All library errors derive from `velox::errors::VeloxError` (specializations include `ConfigError`, `InputError`, and `SimulationError`). Errors include contextual file/line metadata via the `VELOX_LOC`/`VELOX_MODEL`/`VELOX_CONTEXT` helpers.
- User input validation throws `InputError`; configuration and simulation failures throw `ConfigError` or `SimulationError` respectively. Callers should surface these messages to users and stop stepping until resolved.
- The daemon accepts an optional `logging::LogSinkPtr` to route warnings about clamping, timing adjustments, or controller limits to a host-provided logger.

To see clamp/limit warnings without wiring a logger, enable the default console sink on construction:

```cpp
velox::simulation::SimulationDaemon::InitParams init{};
init.use_default_log_sink();
velox::simulation::SimulationDaemon daemon(init);
```

## Building

```bash
cmake -S . -B build -DBUILD_VELOX_EXAMPLES=ON
cmake --build build
```

Targets:
- `velox` – static library containing the daemon, controllers, telemetry, and utility code.
- `commonroad_app` – SDL2/ImGui demo that showcases migrating a legacy UI to the daemon API.
- `basic_sim_daemon` – minimal example under `examples/` showing how to step the daemon and consume telemetry.
- `drift_mode_demo` – headless example that flips the drift safety profile on/off to demonstrate oversteer allowances.

## SDL demo controls

- **Throttle/Brake**: `W`/`↑` for throttle, `S`/`↓` for proportional braking (scaled by “Keyboard brake bias”), `Space` for full braking.
- **Steering**: `A`/`←` left, `D`/`→` right.
- **Reset**: `R` resets the simulator to the current model/vehicle; `Esc` quits.

The demo now relies exclusively on `SimulationDaemon`: all controller wiring, timing sub-steps, telemetry accumulation, and safety enforcement flow through the daemon, mirroring how legacy apps can migrate to the library.
