"""Numerical integration helpers for vehicle models."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Sequence, Tuple

from .low_speed_safety import LowSpeedSafety


State = List[float]


def _as_state(values: Sequence[float]) -> State:
    return [float(v) for v in values]


def _ensure_state_list(state: Sequence[float]) -> State:
    if isinstance(state, list):
        arr = state
        for idx, value in enumerate(arr):
            arr[idx] = float(value)
        return arr
    return [float(v) for v in state]


def _add_scaled(base: Sequence[float], scale: float, delta: Sequence[float]) -> State:
    return [float(b + scale * d) for b, d in zip(base, delta)]


@dataclass(frozen=True)
class ModelInterface:
    """Minimal interface required to integrate a vehicle model."""

    init_fn: Callable[[Sequence[float], object], Sequence[float]]
    dynamics_fn: Callable[[Sequence[float], Sequence[float], object], Sequence[float]]
    speed_fn: Callable[[Sequence[float]], float]


class VehicleSimulator:
    """Runge-Kutta integrator with low-speed safeguards."""

    def __init__(
        self,
        model: ModelInterface,
        params: object,
        dt: float,
        safety: LowSpeedSafety,
    ) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        self._model = model
        self._params = params
        self._dt = dt
        self._safety = safety
        self._state: State | None = None

    @property
    def state(self) -> State:
        if self._state is None:
            raise RuntimeError("VehicleSimulator has not been initialised; call reset() first")
        return list(self._state)

    def reset(self, initial_state: Sequence[float]) -> None:
        self._safety.reset()
        state = self._model.init_fn(initial_state, self._params)
        self._state = self._apply_safety_inplace(_ensure_state_list(state))

    def speed(self) -> float:
        if self._state is None:
            raise RuntimeError("VehicleSimulator has not been initialised; call reset() first")
        return float(self._model.speed_fn(self._state))

    def step(self, control: Sequence[float]) -> State:
        if self._state is None:
            raise RuntimeError("VehicleSimulator has not been initialised; call reset() first")
        if len(control) != 2:
            raise ValueError("control must contain steering rate and longitudinal acceleration")

        dt = self._dt
        u = [float(control[0]), float(control[1])]
        current = self._state

        k1, current = self._dynamics(current, u)
        self._state = current

        k2_state = _add_scaled(current, 0.5 * dt, k1)
        k2, _ = self._dynamics(k2_state, u)

        k3_state = _add_scaled(current, 0.5 * dt, k2)
        k3, _ = self._dynamics(k3_state, u)

        k4_state = _add_scaled(current, dt, k3)
        k4, _ = self._dynamics(k4_state, u)

        new_state = [
            current[i]
            + (dt / 6.0)
            * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
            for i in range(len(current))
        ]
        self._state = self._apply_safety_inplace(new_state)
        return self.state

    # Internal helpers -----------------------------------------------------
    def _dynamics(self, state: Sequence[float], control: Sequence[float]) -> Tuple[State, State]:
        arr = self._apply_safety_inplace(state)
        rhs = self._model.dynamics_fn(arr, control, self._params)
        return _as_state(rhs), arr

    def _apply_safety_inplace(self, state: Sequence[float]) -> State:
        arr = _ensure_state_list(state)
        speed = float(self._model.speed_fn(arr))
        self._safety.apply(arr, speed)
        return arr


__all__ = ["ModelInterface", "VehicleSimulator"]
