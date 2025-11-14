"""Numerical integration helpers for vehicle models."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Sequence

from .low_speed_safety import LowSpeedSafety


State = List[float]


def _as_state(values: Sequence[float]) -> State:
    return [float(v) for v in values]


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
        self._state = self._apply_safety(state)

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
        s = self._state

        k1 = self._dynamics(s, u)
        k2 = self._dynamics(_add_scaled(s, 0.5 * dt, k1), u)
        k3 = self._dynamics(_add_scaled(s, 0.5 * dt, k2), u)
        k4 = self._dynamics(_add_scaled(s, dt, k3), u)

        new_state = [
            s[i]
            + (dt / 6.0)
            * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
            for i in range(len(s))
        ]
        self._state = self._apply_safety(new_state)
        return self.state

    # Internal helpers -----------------------------------------------------
    def _dynamics(self, state: Sequence[float], control: Sequence[float]) -> State:
        clamped_state = self._apply_safety(state)
        rhs = self._model.dynamics_fn(clamped_state, control, self._params)
        return _as_state(rhs)

    def _apply_safety(self, state: Sequence[float]) -> State:
        arr = _as_state(state)
        speed = float(self._model.speed_fn(arr))
        self._safety.apply(arr, speed)
        return arr


__all__ = ["ModelInterface", "VehicleSimulator"]
