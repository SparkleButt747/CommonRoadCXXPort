from __future__ import annotations

import pathlib
import sys

import pytest

PYTHON_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.sim.actuators.aero import AeroConfig
from vehiclemodels.sim.actuators.brake import BrakeConfig
from vehiclemodels.sim.actuators.powertrain import PowertrainConfig
from vehiclemodels.sim.actuators.rolling_resistance import RollingResistanceConfig
from vehiclemodels.sim.controllers.final_accel_controller import (
    DriverIntent,
    FinalAccelController,
    FinalAccelControllerConfig,
)

CONFIG_ROOT = PYTHON_ROOT / "vehiclemodels" / "config"


def _load_cfg(component: str) -> dict:
    path = CONFIG_ROOT / component / "vehicle2.yaml"
    if not path.exists():
        raise FileNotFoundError(path)
    import omegaconf

    return omegaconf.OmegaConf.to_object(omegaconf.OmegaConf.load(path))


def _build_controller(
    accel_max: float = 4.0,
    accel_min: float = -8.0,
    stop_speed_epsilon: float = 0.05,
) -> FinalAccelController:
    params = parameters_vehicle2()
    return FinalAccelController(
        vehicle_mass=params.m,
        wheel_radius=params.R_w,
        powertrain_cfg=PowertrainConfig(**_load_cfg("powertrain")),
        aero_cfg=AeroConfig(**_load_cfg("aero")),
        rolling_cfg=RollingResistanceConfig(**_load_cfg("rolling_resistance")),
        brake_cfg=BrakeConfig(**_load_cfg("brakes")),
        controller_cfg=FinalAccelControllerConfig(
            tau_throttle=0.05,
            tau_brake=0.05,
            accel_min=accel_min,
            accel_max=accel_max,
            stop_speed_epsilon=stop_speed_epsilon,
        ),
    )


def test_steady_cruise_zero_intent() -> None:
    controller = _build_controller()
    output = controller.step(DriverIntent(throttle=0.0, brake=0.0), speed=0.0, dt=0.1)
    assert output.acceleration == pytest.approx(0.0, abs=1e-6)
    assert output.drive_force == pytest.approx(0.0, abs=1e-6)
    assert output.brake_force == pytest.approx(0.0, abs=1e-6)


def test_full_throttle_respects_limit() -> None:
    accel_limit = 0.6
    controller = _build_controller(accel_max=accel_limit)
    output = None
    for _ in range(20):
        output = controller.step(DriverIntent(throttle=1.0, brake=0.0), speed=0.0, dt=0.05)
    assert output is not None
    assert output.acceleration == pytest.approx(accel_limit, rel=1e-2)
    assert output.drive_force > 0.0
    assert output.brake_force == pytest.approx(0.0, abs=1e-3)


def test_blended_braking_near_regen_cutoff() -> None:
    controller = _build_controller()
    brake_cfg = BrakeConfig(**_load_cfg("brakes"))
    speed_above = brake_cfg.min_regen_speed + 0.5
    speed_below = max(0.0, brake_cfg.min_regen_speed - 0.5)

    output_above = None
    for _ in range(10):
        output_above = controller.step(DriverIntent(throttle=0.0, brake=1.0), speed=speed_above, dt=0.05)
    assert output_above is not None
    assert output_above.regen_force > 0.0
    assert output_above.hydraulic_force < output_above.brake_force

    controller_low = _build_controller()
    output_below = None
    for _ in range(10):
        output_below = controller_low.step(DriverIntent(throttle=0.0, brake=1.0), speed=speed_below, dt=0.05)
    assert output_below is not None
    assert output_below.regen_force == pytest.approx(0.0, abs=1e-6)
    assert output_below.hydraulic_force == pytest.approx(output_below.brake_force, rel=1e-3)


def test_braking_does_not_drive_backwards_at_rest() -> None:
    controller = _build_controller()
    output = None
    for _ in range(10):
        output = controller.step(DriverIntent(throttle=0.0, brake=1.0), speed=0.0, dt=0.05)
    assert output is not None
    assert output.acceleration >= 0.0


def test_reset_restores_integrators_and_soc() -> None:
    controller = _build_controller()
    intent = DriverIntent(throttle=1.0, brake=0.0)
    for _ in range(20):
        controller.step(intent, speed=10.0, dt=0.1)

    initial_soc = controller.powertrain.config.initial_soc
    assert controller.throttle > 0.0
    assert controller.powertrain.soc < initial_soc

    controller.reset()

    assert controller.throttle == pytest.approx(0.0)
    assert controller.brake == pytest.approx(0.0)
    assert controller.powertrain.soc == pytest.approx(initial_soc)
