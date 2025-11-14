from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass(frozen=True)
class SteeringWheelConfig:
    """Configuration for the high-level virtual steering wheel."""

    max_angle: float
    max_rate: float
    nudge_angle: float
    centering_stiffness: float
    centering_deadband: float

    def __post_init__(self) -> None:  # type: ignore[override]
        if self.max_angle <= 0.0:
            raise ValueError("max_angle must be positive")
        if self.max_rate <= 0.0:
            raise ValueError("max_rate must be positive")
        if self.nudge_angle <= 0.0:
            raise ValueError("nudge_angle must be positive")
        if self.centering_stiffness <= 0.0:
            raise ValueError("centering_stiffness must be positive")
        if self.centering_deadband < 0.0:
            raise ValueError("centering_deadband cannot be negative")
        if self.centering_deadband >= self.max_angle:
            raise ValueError("centering_deadband must be smaller than max_angle")


class SteeringWheel:
    """Discrete-time model of a hand wheel with self-centering."""

    def __init__(self, config: SteeringWheelConfig, initial_angle: float = 0.0) -> None:
        self._config = config
        self._angle = 0.0
        self.reset(initial_angle)

    @property
    def angle(self) -> float:
        return self._angle

    def reset(self, angle: float = 0.0) -> None:
        limit = self._config.max_angle
        self._angle = float(max(-limit, min(limit, angle)))

    def update(self, nudge: float, dt: float) -> float:
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        cfg = self._config
        nudge = max(-1.0, min(1.0, nudge))
        angle = self._angle

        if nudge != 0.0:
            # Discrete nudge towards left/right bounded by configured rate and amplitude.
            target = angle + nudge * cfg.nudge_angle
            target = max(-cfg.max_angle, min(cfg.max_angle, target))
            max_delta = cfg.max_rate * dt
            delta = target - angle
            delta = max(-max_delta, min(max_delta, delta))
            angle += delta
        else:
            # Automatic centering behaves like a first-order spring pulling towards zero.
            if abs(angle) <= cfg.centering_deadband:
                angle = 0.0
            else:
                # Shift the effective error so the deadband becomes a flat zone around zero.
                effective_angle = math.copysign(
                    max(0.0, abs(angle) - cfg.centering_deadband), angle
                )
                rate = -cfg.centering_stiffness * effective_angle
                rate = max(-cfg.max_rate, min(cfg.max_rate, rate))
                angle += rate * dt
                # Prevent overshooting through zero in a single step.
                if angle == 0.0 or math.copysign(1.0, angle) != math.copysign(1.0, self._angle):
                    angle = 0.0

        # Clamp to the physical limits in case integration error accrued.
        self._angle = max(-cfg.max_angle, min(cfg.max_angle, angle))
        return self._angle


@dataclass(frozen=True)
class FinalSteerControllerConfig:
    """Configuration for the low-level steering actuator model."""

    min_angle: float
    max_angle: float
    max_rate: float
    actuator_time_constant: float
    smoothing_time_constant: float

    def __post_init__(self) -> None:  # type: ignore[override]
        if self.max_angle <= self.min_angle:
            raise ValueError("max_angle must be greater than min_angle")
        if self.max_rate <= 0.0:
            raise ValueError("max_rate must be positive")
        if self.actuator_time_constant <= 0.0:
            raise ValueError("actuator_time_constant must be positive")
        if self.smoothing_time_constant < 0.0:
            raise ValueError("smoothing_time_constant cannot be negative")


class FinalSteerController:
    """Rate-limited steering actuator with first-order lag and smoothing."""

    def __init__(
        self,
        config: FinalSteerControllerConfig,
        initial_angle: float = 0.0,
    ) -> None:
        self._config = config
        clamped = self._clamp(initial_angle)
        self._angle = clamped
        self._filtered_command = clamped

    @property
    def angle(self) -> float:
        return self._angle

    def reset(self, angle: float = 0.0) -> None:
        clamped = self._clamp(angle)
        self._angle = clamped
        self._filtered_command = clamped

    def step(self, desired_angle: float, measured_angle: float, dt: float) -> Tuple[float, float]:
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        cfg = self._config
        desired = self._clamp(desired_angle)
        measured = self._clamp(measured_angle)

        # Low-pass filter on the operator command to avoid abrupt jumps.
        if cfg.smoothing_time_constant > 0.0:
            alpha = dt / (cfg.smoothing_time_constant + dt)
            self._filtered_command += alpha * (desired - self._filtered_command)
        else:
            self._filtered_command = desired

        # First-order actuator dynamics with a hard rate limit.
        error = self._filtered_command - measured
        tau = cfg.actuator_time_constant
        rate = error / tau
        rate = max(-cfg.max_rate, min(cfg.max_rate, rate))

        # Respect physical steering stops even if the plant is already near a limit.
        max_step = (cfg.max_angle - measured) / dt
        min_step = (cfg.min_angle - measured) / dt
        rate = max(min_step, min(max_step, rate))

        new_angle = measured + rate * dt
        new_angle = self._clamp(new_angle)

        if dt > 0.0:
            rate = (new_angle - measured) / dt
        self._angle = new_angle
        return self._angle, rate

    def _clamp(self, angle: float) -> float:
        cfg = self._config
        return max(cfg.min_angle, min(cfg.max_angle, angle))


__all__ = [
    "SteeringWheelConfig",
    "SteeringWheel",
    "FinalSteerControllerConfig",
    "FinalSteerController",
]
