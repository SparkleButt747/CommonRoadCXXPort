import pathlib
import sys
from typing import Iterable, Tuple

from omegaconf import OmegaConf

PYTHON_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from vehiclemodels.init_st import init_st
from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.sim import LowSpeedSafety, LowSpeedSafetyConfig, ModelInterface, VehicleSimulator
from vehiclemodels.sim.actuators.aero import AeroConfig
from vehiclemodels.sim.actuators.brake import BrakeConfig
from vehiclemodels.sim.actuators.powertrain import PowertrainConfig
from vehiclemodels.sim.actuators.rolling_resistance import RollingResistanceConfig
from vehiclemodels.sim.controllers.final_accel_controller import (
    DriverIntent,
    FinalAccelController,
    FinalAccelControllerConfig,
)
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st


CONFIG_ROOT = PYTHON_ROOT / "vehiclemodels" / "config"


def _load_component(component: str) -> dict:
    path = CONFIG_ROOT / component / "vehicle2.yaml"
    if not path.exists():
        path = CONFIG_ROOT / component / "default.yaml"
    if not path.exists():
        raise FileNotFoundError(path)
    return OmegaConf.to_object(OmegaConf.load(path))


def _load_low_speed_cfg() -> LowSpeedSafetyConfig:
    data = _load_component("low_speed_safety")
    return LowSpeedSafetyConfig(**data)


def _build_controller(stop_speed_epsilon: float) -> FinalAccelController:
    params = parameters_vehicle2()
    return FinalAccelController(
        vehicle_mass=params.m,
        wheel_radius=params.R_w,
        powertrain_cfg=PowertrainConfig(**_load_component("powertrain")),
        aero_cfg=AeroConfig(**_load_component("aero")),
        rolling_cfg=RollingResistanceConfig(**_load_component("rolling_resistance")),
        brake_cfg=BrakeConfig(**_load_component("brakes")),
        controller_cfg=FinalAccelControllerConfig(
            tau_throttle=0.1,
            tau_brake=0.1,
            accel_min=-8.0,
            accel_max=4.0,
            stop_speed_epsilon=stop_speed_epsilon,
        ),
    )


def _phases() -> Iterable[Tuple[float, DriverIntent]]:
    return [
        (1.5, DriverIntent(throttle=1.0, brake=0.0)),
        (1.5, DriverIntent(throttle=0.0, brake=1.0)),
        (0.5, DriverIntent(throttle=0.0, brake=0.0)),
        (1.0, DriverIntent(throttle=1.0, brake=0.0)),
    ]


def test_stop_and_go_remains_stable() -> None:
    params = parameters_vehicle2()
    cfg = _load_low_speed_cfg()
    safety = LowSpeedSafety(
        cfg,
        longitudinal_index=3,
        yaw_rate_index=5,
        slip_index=6,
    )

    model = ModelInterface(
        init_fn=lambda state, _params: init_st(list(state)),
        dynamics_fn=vehicle_dynamics_st,
        speed_fn=lambda state: float(state[3]),
    )
    dt = 0.01
    simulator = VehicleSimulator(model, params, dt=dt, safety=safety)
    simulator.reset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    controller = _build_controller(cfg.stop_speed_epsilon)

    speeds = []
    yaw_rates = []
    slips = []

    for duration, intent in _phases():
        steps = int(duration / dt)
        for _ in range(steps):
            output = controller.step(intent, speed=simulator.speed(), dt=dt)
            state = simulator.step((0.0, output.acceleration))
            speeds.append(simulator.speed())
            yaw_rates.append(state[5])
            slips.append(state[6])

    assert speeds, "simulation produced no samples"
    min_speed = min(speeds)
    max_speed = max(speeds)
    assert min_speed >= -1e-6
    assert max_speed > 1.0

    engaged_samples = [idx for idx, v in enumerate(speeds) if v < cfg.engage_speed + 1e-3]
    assert engaged_samples, "vehicle never reached low-speed regime"

    tol = 5e-4
    for idx in engaged_samples:
        assert abs(yaw_rates[idx]) <= tol
        assert abs(slips[idx]) <= tol

    assert max(abs(val) for val in yaw_rates) <= cfg.yaw_rate_limit + 1e-3
    assert max(abs(val) for val in slips) <= cfg.slip_angle_limit + 1e-3
