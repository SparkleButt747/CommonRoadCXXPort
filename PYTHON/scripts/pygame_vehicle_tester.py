"""Interactive manual testing harness for CommonRoad vehicle models using Pygame.

This script lets a user drive the reference Python vehicle dynamics models with a
keyboard to get an intuitive feeling for their behavior.  Choose the model and
parameter set in the CONFIG dictionary below and launch the script with

    python PYTHON/scripts/pygame_vehicle_tester.py

(When running from the repository root you may want to prepend
``PYTHONPATH=PYTHON:.`` so the vehiclemodels package can be imported.)
"""

from __future__ import annotations

import logging
import math
import pathlib
import sys
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import pygame
import numpy as np
from omegaconf import OmegaConf

# Ensure the vehiclemodels package is importable when the script is executed
# from arbitrary working directories.
REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
PYTHON_ROOT = REPO_ROOT / "PYTHON"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

# Vehicle parameter factories
from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.parameters_vehicle3 import parameters_vehicle3
from vehiclemodels.parameters_vehicle4 import parameters_vehicle4
from vehiclemodels.sim.actuators.aero import AeroConfig
from vehiclemodels.sim.actuators.brake import BrakeConfig
from vehiclemodels.sim.actuators.powertrain import PowertrainConfig
from vehiclemodels.sim.actuators.rolling_resistance import RollingResistanceConfig
from vehiclemodels.sim import (
    LowSpeedSafety,
    LowSpeedSafetyConfig,
    ModelInterface,
    VehicleSimulator as CoreSimulator,
)
from vehiclemodels.sim.controllers.final_accel_controller import (
    ControllerOutput,
    DriverIntent,
    FinalAccelController,
    FinalAccelControllerConfig,
)
from vehiclemodels.sim.controllers.steering import (
    FinalSteerController,
    FinalSteerControllerConfig,
    SteeringWheel,
    SteeringWheelConfig,
)

# Vehicle dynamics and state initializers
from vehiclemodels.init_ks import init_ks
from vehiclemodels.init_st import init_st
from vehiclemodels.init_std import init_std
from vehiclemodels.init_mb import init_mb
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std
from vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb


@dataclass(frozen=True)
class ModelSpec:
    """Metadata required to simulate and visualise a vehicle model."""

    name: str
    init_fn: Callable[[Sequence[float], object], Sequence[float]]
    dynamics_fn: Callable[[Sequence[float], Sequence[float], object], Sequence[float]]
    position_fn: Callable[[Sequence[float]], Tuple[float, float]]
    yaw_fn: Callable[[Sequence[float]], float]
    speed_fn: Callable[[Sequence[float]], float]
    longitudinal_index: Optional[int]
    yaw_rate_index: Optional[int]
    slip_index: Optional[int]


# Model-specific helper lambdas -------------------------------------------------

def _init_ks(state: Sequence[float], _params: object) -> Sequence[float]:
    return init_ks(list(state))


def _init_st(state: Sequence[float], _params: object) -> Sequence[float]:
    return list(init_st(list(state)))


def _init_std(state: Sequence[float], params: object) -> Sequence[float]:
    return list(init_std(list(state), params))


def _init_mb(state: Sequence[float], params: object) -> Sequence[float]:
    return list(init_mb(list(state), params))


def _pos_xy0(state: Sequence[float]) -> Tuple[float, float]:
    return float(state[0]), float(state[1])


def _yaw_idx(index: int) -> Callable[[Sequence[float]], float]:
    def _getter(state: Sequence[float]) -> float:
        return float(state[index])

    return _getter


def _speed_single_track(state: Sequence[float]) -> float:
    # Longitudinal velocity in the model's body frame is stored in x[3].
    return float(state[3])


def _speed_multi_body(state: Sequence[float]) -> float:
    # Combine longitudinal and lateral velocities in the vehicle frame.
    v_long = float(state[3])
    v_lat = float(state[10])
    return math.hypot(v_long, v_lat)


MODEL_SPECS: Dict[str, ModelSpec] = {
    "ks": ModelSpec(
        name="Kinematic Single Track",
        init_fn=_init_ks,
        dynamics_fn=vehicle_dynamics_ks,
        position_fn=_pos_xy0,
        yaw_fn=_yaw_idx(4),
        speed_fn=_speed_single_track,
        longitudinal_index=3,
        yaw_rate_index=None,
        slip_index=None,
    ),
    "st": ModelSpec(
        name="Dynamic Single Track",
        init_fn=_init_st,
        dynamics_fn=vehicle_dynamics_st,
        position_fn=_pos_xy0,
        yaw_fn=_yaw_idx(4),
        speed_fn=_speed_single_track,
        longitudinal_index=3,
        yaw_rate_index=5,
        slip_index=6,
    ),
    "std": ModelSpec(
        name="Single Track Drift",
        init_fn=_init_std,
        dynamics_fn=vehicle_dynamics_std,
        position_fn=_pos_xy0,
        yaw_fn=_yaw_idx(4),
        speed_fn=_speed_single_track,
        longitudinal_index=3,
        yaw_rate_index=5,
        slip_index=6,
    ),
    "mb": ModelSpec(
        name="Multi Body",
        init_fn=_init_mb,
        dynamics_fn=vehicle_dynamics_mb,
        position_fn=_pos_xy0,
        yaw_fn=_yaw_idx(4),
        speed_fn=_speed_multi_body,
        longitudinal_index=3,
        yaw_rate_index=5,
        slip_index=None,
    ),
}


PARAMETER_FACTORIES = {
    "vehicle1": parameters_vehicle1,
    "vehicle2": parameters_vehicle2,
    "vehicle3": parameters_vehicle3,
    "vehicle4": parameters_vehicle4,
}


# ------------------------------------------------------------------------------
# User-facing configuration block
CONFIG = {
    "model": "st",  # one of: "ks", "st", "std", "mb"
    "parameter_set": "vehicle2",  # one of: vehicle1..vehicle4
    "time_step": 0.01,  # integration step size in seconds
    "max_accel": 4.0,  # [m/s^2] upper bound for longitudinal acceleration
    "max_brake": 8.0,  # [m/s^2] peak braking deceleration (positive magnitude)
    "history_length": 2000,  # number of past poses to keep for drawing the path
    "window_size": (960, 720),
    "meters_per_pixel": 0.5,  # scale factor for rendering (smaller -> zoomed out)
    "steering_config": PYTHON_ROOT / "config" / "steering.yaml",
}

CONFIG_ROOT = PYTHON_ROOT / "vehiclemodels" / "config"


def _load_component_config(component: str, parameter_set: str) -> dict:
    base = CONFIG_ROOT / component
    path = base / f"{parameter_set}.yaml"
    if not path.exists():
        path = base / "default.yaml"
    if not path.exists():
        raise FileNotFoundError(f"No config for component '{component}' and parameter set '{parameter_set}'")
    return OmegaConf.to_object(OmegaConf.load(path))


def _build_low_speed_safety(spec: ModelSpec) -> tuple[LowSpeedSafety, LowSpeedSafetyConfig]:
    parameter_set = CONFIG["parameter_set"]
    cfg_dict = _load_component_config("low_speed_safety", parameter_set)
    cfg = LowSpeedSafetyConfig(**cfg_dict)
    safety = LowSpeedSafety(
        cfg,
        longitudinal_index=spec.longitudinal_index,
        yaw_rate_index=spec.yaw_rate_index,
        slip_index=spec.slip_index,
    )
    return safety, cfg


def _build_accel_controller(
    params: object, low_speed_cfg: LowSpeedSafetyConfig
) -> FinalAccelController:
    parameter_set = CONFIG["parameter_set"]
    powertrain_cfg = PowertrainConfig(**_load_component_config("powertrain", parameter_set))
    aero_cfg = AeroConfig(**_load_component_config("aero", parameter_set))
    rolling_cfg = RollingResistanceConfig(**_load_component_config("rolling_resistance", parameter_set))
    brake_cfg = BrakeConfig(**_load_component_config("brakes", parameter_set))

    if not getattr(params, "m", None):
        raise ValueError("Vehicle parameters must define mass 'm'")
    if not getattr(params, "R_w", None):
        raise ValueError("Vehicle parameters must define wheel radius 'R_w'")

    controller_cfg = FinalAccelControllerConfig(
        tau_throttle=0.25,
        tau_brake=0.15,
        accel_min=-CONFIG["max_brake"],
        accel_max=CONFIG["max_accel"],
        stop_speed_epsilon=low_speed_cfg.stop_speed_epsilon,
    )

    return FinalAccelController(
        vehicle_mass=params.m,
        wheel_radius=params.R_w,
        powertrain_cfg=powertrain_cfg,
        aero_cfg=aero_cfg,
        rolling_cfg=rolling_cfg,
        brake_cfg=brake_cfg,
        controller_cfg=controller_cfg,
    )

# Initial state shared across models: [sx, sy, delta, vel, psi, yaw_rate, slip]
BASE_INITIAL_STATE = [0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0]


class VehicleSimulator:
    def __init__(self, spec: ModelSpec, params: object, dt: float, safety: LowSpeedSafety):
        self.spec = spec
        self.params = params
        self.dt = dt
        self._core = CoreSimulator(
            model=ModelInterface(
                init_fn=spec.init_fn,
                dynamics_fn=spec.dynamics_fn,
                speed_fn=spec.speed_fn,
            ),
            params=params,
            dt=dt,
            safety=safety,
        )
        self.history: List[Tuple[float, float]] = []
        self.state: List[float] = []
        self.reset()

    def reset(self) -> None:
        self._core.reset(BASE_INITIAL_STATE)
        self.state = self._core.state
        self.history.clear()

    def step(self, steer_rate: float, acceleration: float) -> None:
        state = self._core.step((steer_rate, acceleration))
        self.state = state

        pos = self.spec.position_fn(state)
        self.history.append(pos)
        if len(self.history) > CONFIG["history_length"]:
            self.history = self.history[-CONFIG["history_length"] :]

    # Convenience getters ----------------------------------------------------
    @property
    def pose(self) -> Tuple[float, float, float]:
        x, y = self.spec.position_fn(self.state)
        psi = self.spec.yaw_fn(self.state)
        return x, y, psi

    @property
    def speed(self) -> float:
        return self._core.speed()

    @property
    def steering_angle(self) -> float:
        # All supported vehicle models expose the front wheel steering angle in state[2].
        return float(self.state[2])


def _handle_input() -> Tuple[int, DriverIntent]:
    keys = pygame.key.get_pressed()
    steer_nudge = 0
    throttle = 0.0
    brake = 0.0

    if keys[pygame.K_UP]:
        throttle = 1.0
    if keys[pygame.K_DOWN]:
        brake = max(brake, 0.5)
    if keys[pygame.K_SPACE]:
        brake = 1.0

    if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
        steer_nudge = -1
    elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
        steer_nudge = 1

    return steer_nudge, DriverIntent(throttle=throttle, brake=brake)


def _draw_vehicle(
    screen: pygame.Surface,
    simulator: VehicleSimulator,
    font: pygame.font.Font,
    ctrl_output: "ControllerOutput | None",
    commanded_angle: float,
    commanded_rate: float,
    desired_angle: float,
    applied_accel: float,
    actual_angle: float,
    actual_rate: float,
) -> None:
    width, height = screen.get_size()
    cx, cy = width // 2, height // 2
    scale = 1.0 / CONFIG["meters_per_pixel"]

    screen.fill((20, 20, 20))

    # Draw path history
    if simulator.history:
        points = [
            (int(cx + px * scale), int(cy - py * scale))
            for px, py in simulator.history
        ]
        if len(points) > 1:
            pygame.draw.lines(screen, (70, 150, 70), False, points, 2)

    # Draw vehicle body as a simple triangle pointing forwards
    x, y, psi = simulator.pose
    px = cx + x * scale
    py = cy - y * scale
    vehicle_length = 4.5  # metres
    vehicle_width = 1.8
    half_w = vehicle_width / 2.0

    heading = np.array([math.cos(psi), math.sin(psi)])
    left = np.array([-heading[1], heading[0]])

    nose = np.array([x, y]) + heading * vehicle_length * 0.5
    tail = np.array([x, y]) - heading * vehicle_length * 0.5
    left_corner = tail + left * half_w
    right_corner = tail - left * half_w

    triangle = [
        (int(cx + nose[0] * scale), int(cy - nose[1] * scale)),
        (int(cx + left_corner[0] * scale), int(cy - left_corner[1] * scale)),
        (int(cx + right_corner[0] * scale), int(cy - right_corner[1] * scale)),
    ]
    pygame.draw.polygon(screen, (200, 200, 40), triangle)
    pygame.draw.circle(screen, (240, 240, 240), (int(px), int(py)), 4)

    text_lines = [
        f"Model: {CONFIG['model']} ({MODEL_SPECS[CONFIG['model']].name})",
        f"Parameters: {CONFIG['parameter_set']}",
        f"Speed: {simulator.speed:5.2f} m/s",
        f"Steer angle (vehicle): {math.degrees(actual_angle):5.2f} deg",
        f"Steer angle (cmd est): {math.degrees(commanded_angle):5.2f} deg",
        f"Steer rate cmd: {math.degrees(commanded_rate):5.2f} deg/s",
        f"Steer rate actual: {math.degrees(actual_rate):5.2f} deg/s",
        f"Desired angle: {math.degrees(desired_angle):5.2f} deg",
        f"Applied accel: {applied_accel:5.2f} m/s^2",
    ]
    if ctrl_output is not None:
        text_lines.extend(
            [
                f"Throttle: {ctrl_output.throttle:4.2f}",
                f"Brake: {ctrl_output.brake:4.2f}",
            ]
        )
    text_lines.extend(
        [
            "Controls: arrows steer/throttle, SPACE brake hard, R reset",
            "ESC to quit",
        ]
    )
    for i, line in enumerate(text_lines):
        surface = font.render(line, True, (220, 220, 220))
        screen.blit(surface, (10, 10 + 18 * i))


def _validate_config() -> Tuple[ModelSpec, object]:
    if CONFIG["model"] not in MODEL_SPECS:
        raise ValueError(f"Unknown model '{CONFIG['model']}'. Options: {sorted(MODEL_SPECS)}")
    if CONFIG["parameter_set"] not in PARAMETER_FACTORIES:
        raise ValueError(
            f"Unknown parameter set '{CONFIG['parameter_set']}'. Options: {sorted(PARAMETER_FACTORIES)}"
        )
    spec = MODEL_SPECS[CONFIG["model"]]
    params = PARAMETER_FACTORIES[CONFIG["parameter_set"]]()
    return spec, params


def main() -> None:
    spec, params = _validate_config()

    steering_cfg_path = pathlib.Path(CONFIG["steering_config"])
    if not steering_cfg_path.exists():
        raise FileNotFoundError(f"Steering config not found: {steering_cfg_path}")
    steering_cfg_data = OmegaConf.to_object(OmegaConf.load(steering_cfg_path))
    if "wheel" not in steering_cfg_data or "final" not in steering_cfg_data:
        raise ValueError("Steering config must define 'wheel' and 'final' sections")
    wheel = SteeringWheel(SteeringWheelConfig(**steering_cfg_data["wheel"]))
    steer_controller = FinalSteerController(
        FinalSteerControllerConfig(**steering_cfg_data["final"])
    )

    pygame.init()
    pygame.display.set_caption("CommonRoad Vehicle Model Playground")
    screen = pygame.display.set_mode(CONFIG["window_size"])
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    safety, low_speed_cfg = _build_low_speed_safety(spec)
    simulator = VehicleSimulator(spec, params, dt=CONFIG["time_step"], safety=safety)
    accel_controller = _build_accel_controller(params, low_speed_cfg)
    fps = max(1, int(round(1.0 / CONFIG["time_step"])))

    running = True
    controller_output: ControllerOutput | None = None
    last_desired_angle = 0.0
    measured_angle = simulator.steering_angle
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                simulator.reset()
                controller_output = None
                wheel.reset()
                steer_controller.reset()
                last_desired_angle = 0.0
                measured_angle = simulator.steering_angle

        steer_nudge, intent = _handle_input()
        last_desired_angle = wheel.update(steer_nudge, CONFIG["time_step"])
        commanded_angle, steer_rate = steer_controller.step(
            last_desired_angle, measured_angle, CONFIG["time_step"]
        )
        controller_output = accel_controller.step(intent, simulator.speed, CONFIG["time_step"])
        simulator.step(steer_rate, controller_output.acceleration)
        actual_angle = simulator.steering_angle
        actual_rate = (actual_angle - measured_angle) / CONFIG["time_step"]
        measured_angle = actual_angle
        applied_accel = controller_output.acceleration if controller_output else 0.0
        _draw_vehicle(
            screen,
            simulator,
            font,
            controller_output,
            commanded_angle,
            steer_rate,
            last_desired_angle,
            applied_accel,
            actual_angle,
            actual_rate,
        )

        pygame.display.flip()
        clock.tick(fps)

    pygame.quit()


if __name__ == "__main__":
    main()
logging.basicConfig(level=logging.INFO)
LOGGER = logging.getLogger(__name__)

