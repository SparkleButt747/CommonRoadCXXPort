"""Interactive manual testing harness for CommonRoad vehicle models using Pygame.

This script lets a user drive the reference Python vehicle dynamics models with a
keyboard to get an intuitive feeling for their behavior.  Choose the model and
parameter set in the CONFIG dictionary below and launch the script with

    python PYTHON/scripts/pygame_vehicle_tester.py

(When running from the repository root you may want to prepend
``PYTHONPATH=PYTHON:.`` so the vehiclemodels package can be imported.)
"""

from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import dataclass
from typing import Callable, Dict, List, Sequence, Tuple

import pygame
import numpy as np

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
    ),
    "st": ModelSpec(
        name="Dynamic Single Track",
        init_fn=_init_st,
        dynamics_fn=vehicle_dynamics_st,
        position_fn=_pos_xy0,
        yaw_fn=_yaw_idx(4),
        speed_fn=_speed_single_track,
    ),
    "std": ModelSpec(
        name="Single Track Drift",
        init_fn=_init_std,
        dynamics_fn=vehicle_dynamics_std,
        position_fn=_pos_xy0,
        yaw_fn=_yaw_idx(4),
        speed_fn=_speed_single_track,
    ),
    "mb": ModelSpec(
        name="Multi Body",
        init_fn=_init_mb,
        dynamics_fn=vehicle_dynamics_mb,
        position_fn=_pos_xy0,
        yaw_fn=_yaw_idx(4),
        speed_fn=_speed_multi_body,
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
    "max_accel": 4.0,  # [m/s^2] peak longitudinal acceleration when pressing UP
    "max_brake": 8.0,  # [m/s^2] peak braking deceleration when pressing DOWN/SPACE
    "max_steer_rate": math.radians(45.0),  # [rad/s] steering angle rate
    "history_length": 2000,  # number of past poses to keep for drawing the path
    "window_size": (960, 720),
    "meters_per_pixel": 0.5,  # scale factor for rendering (smaller -> zoomed out)
}

# Initial state shared across models: [sx, sy, delta, vel, psi, yaw_rate, slip]
BASE_INITIAL_STATE = [0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0]


class VehicleSimulator:
    def __init__(self, spec: ModelSpec, params: object, dt: float):
        self.spec = spec
        self.params = params
        self.dt = dt
        self.state = np.array(spec.init_fn(BASE_INITIAL_STATE, params), dtype=float)
        self.history: List[Tuple[float, float]] = []

    def reset(self) -> None:
        self.state = np.array(self.spec.init_fn(BASE_INITIAL_STATE, self.params), dtype=float)
        self.history.clear()

    # Numerical integration --------------------------------------------------
    def _dynamics(self, state: np.ndarray, control: Tuple[float, float]) -> np.ndarray:
        steer_rate, acceleration = control
        rhs = self.spec.dynamics_fn(state.tolist(), [steer_rate, acceleration], self.params)
        return np.asarray(rhs, dtype=float)

    def step(self, steer_rate: float, acceleration: float) -> None:
        dt = self.dt
        u = (steer_rate, acceleration)
        s = self.state

        # Runge-Kutta 4th order integration
        k1 = self._dynamics(s, u)
        k2 = self._dynamics(s + 0.5 * dt * k1, u)
        k3 = self._dynamics(s + 0.5 * dt * k2, u)
        k4 = self._dynamics(s + dt * k3, u)
        self.state = s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

        pos = self.spec.position_fn(self.state.tolist())
        self.history.append(pos)
        if len(self.history) > CONFIG["history_length"]:
            self.history = self.history[-CONFIG["history_length"] :]

    # Convenience getters ----------------------------------------------------
    @property
    def pose(self) -> Tuple[float, float, float]:
        x, y = self.spec.position_fn(self.state.tolist())
        psi = self.spec.yaw_fn(self.state.tolist())
        return x, y, psi

    @property
    def speed(self) -> float:
        return self.spec.speed_fn(self.state.tolist())


def _handle_input() -> Tuple[float, float]:
    keys = pygame.key.get_pressed()
    accel = 0.0
    steer = 0.0

    if keys[pygame.K_UP]:
        accel += CONFIG["max_accel"]
    if keys[pygame.K_DOWN]:
        accel -= CONFIG["max_brake"]
    if keys[pygame.K_SPACE]:
        accel = -CONFIG["max_brake"]

    if keys[pygame.K_LEFT]:
        steer -= CONFIG["max_steer_rate"]
    if keys[pygame.K_RIGHT]:
        steer += CONFIG["max_steer_rate"]

    return steer, accel


def _draw_vehicle(screen: pygame.Surface, simulator: VehicleSimulator, font: pygame.font.Font) -> None:
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
        "Controls: arrows steer/accelerate, SPACE brake hard, R reset",
        "ESC to quit",
    ]
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

    pygame.init()
    pygame.display.set_caption("CommonRoad Vehicle Model Playground")
    screen = pygame.display.set_mode(CONFIG["window_size"])
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    simulator = VehicleSimulator(spec, params, dt=CONFIG["time_step"])
    fps = max(1, int(round(1.0 / CONFIG["time_step"])))

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                simulator.reset()

        steer, accel = _handle_input()
        simulator.step(steer, accel)
        _draw_vehicle(screen, simulator, font)

        pygame.display.flip()
        clock.tick(fps)

    pygame.quit()


if __name__ == "__main__":
    main()
