from __future__ import annotations

import math
import pathlib
import sys

import pytest

PYTHON_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from vehiclemodels.sim.controllers.steering import (  # noqa: E402
    FinalSteerController,
    FinalSteerControllerConfig,
    SteeringWheel,
    SteeringWheelConfig,
)


@pytest.fixture
def wheel_config() -> SteeringWheelConfig:
    return SteeringWheelConfig(
        max_angle=0.6,
        max_rate=3.5,
        nudge_angle=0.1,
        centering_stiffness=6.0,
        centering_deadband=0.01,
    )


@pytest.fixture
def actuator_config() -> FinalSteerControllerConfig:
    return FinalSteerControllerConfig(
        min_angle=-0.6,
        max_angle=0.6,
        max_rate=2.5,
        actuator_time_constant=0.2,
        smoothing_time_constant=0.1,
    )


def test_wheel_recenters_without_input(wheel_config: SteeringWheelConfig) -> None:
    wheel = SteeringWheel(wheel_config, initial_angle=0.4)
    dt = 0.05
    angle = wheel.angle
    for _ in range(40):
        angle = wheel.update(0.0, dt)
    assert abs(angle) <= wheel_config.centering_deadband + 1e-3


def test_wheel_rate_limit_enforced(wheel_config: SteeringWheelConfig) -> None:
    wheel = SteeringWheel(wheel_config)
    dt = 0.01
    start = wheel.angle
    after = wheel.update(1.0, dt)
    max_delta = wheel_config.max_rate * dt
    assert after - start <= max_delta + 1e-6


def test_final_controller_respects_angle_limits(actuator_config: FinalSteerControllerConfig) -> None:
    controller = FinalSteerController(actuator_config)
    dt = 0.02
    measured = controller.angle
    angle = measured
    rate = 0.0
    for _ in range(200):
        angle, rate = controller.step(1.2, measured, dt)
        measured = angle
    assert angle <= actuator_config.max_angle + 1e-6
    assert angle >= actuator_config.min_angle - 1e-6
    assert math.isfinite(rate)


def test_final_controller_smooths_step_command(actuator_config: FinalSteerControllerConfig) -> None:
    controller = FinalSteerController(actuator_config)
    dt = 0.01
    desired = actuator_config.max_angle * 0.5
    measured = controller.angle
    angle, rate = controller.step(desired, measured, dt)
    assert angle < desired
    assert math.isfinite(rate)
    for _ in range(200):
        measured = angle
        angle, _ = controller.step(desired, measured, dt)
    assert angle == pytest.approx(desired, rel=1e-2)


def test_final_controller_counters_residual_angle(actuator_config: FinalSteerControllerConfig) -> None:
    controller = FinalSteerController(actuator_config)
    dt = 0.05
    measured = 0.1
    angle, rate = controller.step(0.0, measured, dt)
    assert rate < 0.0  # should steer back towards zero
    assert angle < measured
