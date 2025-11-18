import math
import pathlib
import sys
from types import SimpleNamespace
from typing import Iterable, List, Sequence, Tuple

from omegaconf import OmegaConf

PYTHON_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from vehiclemodels.init_st import init_st
from vehiclemodels.init_std import init_std
from vehiclemodels.init_mb import init_mb
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
from vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std
from vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb


CONFIG_ROOT = PYTHON_ROOT / "vehiclemodels" / "config"


def _load_component(component: str) -> dict:
    path = CONFIG_ROOT / component / "vehicle2.yaml"
    if not path.exists():
        path = CONFIG_ROOT / component / "default.yaml"
    if not path.exists():
        raise FileNotFoundError(path)
    data = OmegaConf.to_object(OmegaConf.load(path))
    if component == "powertrain":
        # Validate that the YAML file respects SOC bounds so tests fail fast.
        PowertrainConfig(**data)
    return data


def _load_low_speed_cfg() -> LowSpeedSafetyConfig:
    data = _load_component("low_speed_safety")
    return LowSpeedSafetyConfig(**data)


def _active_profile(cfg: LowSpeedSafetyConfig):
    return cfg.drift if cfg.drift_enabled else cfg.normal


def _speed_single_track(state: Sequence[float]) -> float:
    return float(state[3])


def _speed_multi_body(state: Sequence[float]) -> float:
    return math.hypot(float(state[3]), float(state[10]))


def _run_longitudinal_phases(
    simulator: VehicleSimulator,
    dt: float,
    phases: Iterable[Tuple[float, float]],
) -> Tuple[List[List[float]], List[float]]:
    states: List[List[float]] = []
    speeds: List[float] = []
    for duration, accel in phases:
        steps = int(duration / dt)
        for _ in range(steps):
            state = simulator.step((0.0, accel))
            states.append(list(state))
            speeds.append(simulator.speed())
    return states, speeds


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
    profile = _active_profile(cfg)
    safety = LowSpeedSafety(
        cfg,
        longitudinal_index=3,
        yaw_rate_index=5,
        slip_index=6,
        steering_index=2,
        wheelbase=float(params.a + params.b),
        rear_length=float(params.b),
    )

    model = ModelInterface(
        init_fn=lambda state, _params: init_st(list(state)),
        dynamics_fn=vehicle_dynamics_st,
        speed_fn=_speed_single_track,
    )
    dt = 0.01
    simulator = VehicleSimulator(model, params, dt=dt, safety=safety)
    simulator.reset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    controller = _build_controller(cfg.stop_speed_epsilon)

    speeds: List[float] = []
    yaw_rates: List[float] = []
    slips: List[float] = []
    steering_angles: List[float] = []

    for duration, intent in _phases():
        steps = int(duration / dt)
        for _ in range(steps):
            output = controller.step(intent, speed=simulator.speed(), dt=dt)
            state = simulator.step((0.0, output.acceleration))
            speeds.append(simulator.speed())
            yaw_rates.append(state[5])
            slips.append(state[6])
            steering_angles.append(state[2])

    assert speeds, "simulation produced no samples"
    min_speed = min(speeds)
    max_speed = max(speeds)
    assert min_speed >= -1e-6
    assert max_speed > 1.0

    engaged_samples = [idx for idx, v in enumerate(speeds) if v < profile.engage_speed + 1e-3]
    assert engaged_samples, "vehicle never reached low-speed regime"

    wheelbase = float(params.a + params.b)
    rear_length = float(params.b)
    tol = 5e-3
    for idx in engaged_samples:
        delta = steering_angles[idx]
        beta_ref = math.atan(math.tan(delta) * rear_length / wheelbase)
        expected_yaw = speeds[idx] * math.cos(beta_ref) * math.tan(delta) / wheelbase
        assert math.isfinite(expected_yaw)
        assert abs(yaw_rates[idx] - expected_yaw) <= tol
        assert abs(slips[idx] - beta_ref) <= tol

    assert max(abs(val) for val in yaw_rates) <= profile.yaw_rate_limit + 1e-3
    assert max(abs(val) for val in slips) <= profile.slip_angle_limit + 1e-3


def test_std_resumes_motion_after_full_stop() -> None:
    params = parameters_vehicle2()
    cfg = _load_low_speed_cfg()
    profile = _active_profile(cfg)
    safety = LowSpeedSafety(
        cfg,
        longitudinal_index=3,
        yaw_rate_index=5,
        slip_index=6,
        wheel_speed_indices=(7, 8),
        steering_index=2,
        wheelbase=float(params.a + params.b),
        rear_length=float(params.b),
    )

    model = ModelInterface(
        init_fn=lambda state, vehicle_params: init_std(list(state), vehicle_params),
        dynamics_fn=vehicle_dynamics_std,
        speed_fn=_speed_single_track,
    )
    dt = 0.01
    simulator = VehicleSimulator(model, params, dt=dt, safety=safety)
    simulator.reset([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0])

    phases = [
        (1.0, -6.0),
        (0.75, 0.0),
        (2.0, 2.5),
    ]
    states, speeds = _run_longitudinal_phases(simulator, dt, phases)

    assert speeds, "simulation produced no samples"
    assert any(v < profile.engage_speed + 1e-3 for v in speeds)
    assert min(speeds) >= -1e-6

    accel_steps = int(phases[-1][0] / dt)
    assert max(speeds[-accel_steps:]) > 1.5

    wheel_indices = (7, 8)
    for idx in wheel_indices:
        assert all(state[idx] >= -1e-9 for state in states), f"wheel state {idx} became negative"


def test_std_launch_with_steering_lock_overcomes_latch() -> None:
    params = parameters_vehicle2()
    cfg = _load_low_speed_cfg()
    safety = LowSpeedSafety(
        cfg,
        longitudinal_index=3,
        yaw_rate_index=5,
        slip_index=6,
        wheel_speed_indices=(7, 8),
        steering_index=2,
        wheelbase=float(params.a + params.b),
        rear_length=float(params.b),
    )

    model = ModelInterface(
        init_fn=lambda state, vehicle_params: init_std(list(state), vehicle_params),
        dynamics_fn=vehicle_dynamics_std,
        speed_fn=_speed_single_track,
    )

    dt = 0.01
    simulator = VehicleSimulator(model, params, dt=dt, safety=safety)
    lock_angle = math.radians(30.0)
    simulator.reset([0.0, 0.0, lock_angle, 0.0, 0.0, 0.0, 0.0])

    accel = 3.0
    steps = int(3.0 / dt)
    speeds: List[float] = []
    for _ in range(steps):
        simulator.step((0.0, accel))
        speeds.append(simulator.speed())

    assert speeds, "simulation produced no samples"
    window_start = int(0.5 / dt)
    assert max(speeds[window_start:]) > 1.5
    assert speeds[int(1.5 / dt)] > 0.8
    assert speeds[-1] > 2.0


def test_multi_body_stays_at_rest_after_braking() -> None:
    params = parameters_vehicle2()
    cfg = _load_low_speed_cfg()
    profile = _active_profile(cfg)
    safety = LowSpeedSafety(
        cfg,
        longitudinal_index=3,
        lateral_index=10,
        yaw_rate_index=5,
        slip_index=None,
        wheel_speed_indices=(23, 24, 25, 26),
        steering_index=2,
        wheelbase=float(params.a + params.b),
        rear_length=float(params.b),
    )

    model = ModelInterface(
        init_fn=lambda state, vehicle_params: init_mb(list(state), vehicle_params),
        dynamics_fn=vehicle_dynamics_mb,
        speed_fn=_speed_multi_body,
    )
    dt = 0.005
    simulator = VehicleSimulator(model, params, dt=dt, safety=safety)
    simulator.reset([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0])

    phases = [
        (1.5, -5.0),
        (1.0, 0.0),
        (1.0, 0.0),
    ]
    states, speeds = _run_longitudinal_phases(simulator, dt, phases)

    assert speeds, "simulation produced no samples"
    assert any(v < profile.engage_speed + 1e-3 for v in speeds)

    hold_steps = int(phases[-1][0] / dt)
    hold_speeds = speeds[-hold_steps:]
    assert max(hold_speeds) <= 0.3
    assert max(hold_speeds) - min(hold_speeds) <= 0.05

    wheel_indices = (23, 24, 25, 26)
    for idx in wheel_indices:
        min_value = min(state[idx] for state in states)
        assert min_value >= -1e-9
        hold_values = [state[idx] for state in states[-hold_steps:]]
        assert max(hold_values) <= cfg.stop_speed_epsilon + 1e-3


def test_multi_body_launch_with_steering_lock_overcomes_latch() -> None:
    params = parameters_vehicle2()
    cfg = _load_low_speed_cfg()
    safety = LowSpeedSafety(
        cfg,
        longitudinal_index=3,
        lateral_index=10,
        yaw_rate_index=5,
        slip_index=None,
        wheel_speed_indices=(23, 24, 25, 26),
        steering_index=2,
        wheelbase=float(params.a + params.b),
        rear_length=float(params.b),
    )

    model = ModelInterface(
        init_fn=lambda state, vehicle_params: init_mb(list(state), vehicle_params),
        dynamics_fn=vehicle_dynamics_mb,
        speed_fn=_speed_multi_body,
    )

    dt = 0.005
    simulator = VehicleSimulator(model, params, dt=dt, safety=safety)
    lock_angle = math.radians(30.0)
    simulator.reset([0.0, 0.0, lock_angle, 0.0, 0.0, 0.0, 0.0])

    accel = 3.0
    steps = int(2.5 / dt)
    speeds: List[float] = []
    for _ in range(steps):
        simulator.step((0.0, accel))
        speeds.append(simulator.speed())

    assert speeds, "simulation produced no samples"
    window_start = int(0.5 / dt)
    assert max(speeds[window_start:]) > 1.2
    assert speeds[int(1.5 / dt)] > 0.7
    assert speeds[-1] > 1.4


def test_rk4_predictor_does_not_latch_above_engage_speed() -> None:
    cfg = _load_low_speed_cfg()
    profile = _active_profile(cfg)
    safety = LowSpeedSafety(
        cfg,
        longitudinal_index=0,
        yaw_rate_index=1,
        slip_index=2,
    )

    params = SimpleNamespace(target_speed=profile.engage_speed + 0.005, rate=0.1)

    def init_state(state: Sequence[float], _params: object) -> Sequence[float]:
        return list(state)

    def dynamics(
        state: Sequence[float], _control: Sequence[float], params: SimpleNamespace
    ) -> Sequence[float]:
        speed = float(state[0])
        dvx = -params.rate if speed > params.target_speed else params.rate
        return [dvx, 0.0, 0.0]

    def speed_fn(state: Sequence[float]) -> float:
        return float(state[0])

    model = ModelInterface(
        init_fn=init_state,
        dynamics_fn=dynamics,
        speed_fn=speed_fn,
    )

    dt = 0.2
    simulator = VehicleSimulator(model, params, dt=dt, safety=safety)
    initial_state = [params.target_speed + 0.002, 0.2, 0.1]
    simulator.reset(initial_state)

    state = simulator.step((0.0, 0.0))
    realized_speed = simulator.speed()

    assert realized_speed > profile.engage_speed
    assert not safety.engaged

    yaw_rate = state[1]
    slip = state[2]

    assert abs(yaw_rate - initial_state[1]) < 1e-12
    assert abs(slip - initial_state[2]) < 1e-12
    assert yaw_rate > 0.0
    assert slip > 0.0
