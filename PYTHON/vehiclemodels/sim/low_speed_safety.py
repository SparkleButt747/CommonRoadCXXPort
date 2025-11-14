"""Low-speed stabilisation utilities for vehicle simulations."""
from __future__ import annotations

from dataclasses import dataclass
from typing import MutableSequence, Optional, Sequence


@dataclass(frozen=True)
class LowSpeedSafetyConfig:
    """Configuration thresholds for engaging low-speed safeguards."""

    engage_speed: float
    release_speed: float
    yaw_rate_limit: float
    slip_angle_limit: float
    stop_speed_epsilon: float

    def __post_init__(self) -> None:  # type: ignore[override]
        if self.engage_speed < 0.0:
            raise ValueError("engage_speed must be non-negative")
        if self.release_speed <= 0.0:
            raise ValueError("release_speed must be positive")
        if self.release_speed < self.engage_speed:
            raise ValueError("release_speed must be >= engage_speed")
        if self.yaw_rate_limit <= 0.0:
            raise ValueError("yaw_rate_limit must be positive")
        if self.slip_angle_limit <= 0.0:
            raise ValueError("slip_angle_limit must be positive")
        if self.stop_speed_epsilon < 0.0:
            raise ValueError("stop_speed_epsilon must be non-negative")


class LowSpeedSafety:
    """State limiter to keep vehicle models well-behaved at very low speeds."""

    def __init__(
        self,
        config: LowSpeedSafetyConfig,
        *,
        longitudinal_index: Optional[int],
        lateral_index: Optional[int] = None,
        yaw_rate_index: Optional[int],
        slip_index: Optional[int],
        wheel_speed_indices: Optional[Sequence[int]] = None,
    ) -> None:
        self.config = config
        self._longitudinal_index = longitudinal_index
        self._lateral_index = lateral_index
        self._yaw_rate_index = yaw_rate_index
        self._slip_index = slip_index
        self._wheel_speed_indices: tuple[int, ...] = (
            tuple(int(idx) for idx in wheel_speed_indices)
            if wheel_speed_indices
            else ()
        )
        self._engaged = False

    @property
    def engaged(self) -> bool:
        return self._engaged

    def reset(self) -> None:
        self._engaged = False

    def apply(self, state: MutableSequence[float], speed: float) -> None:
        """Clamp unstable states when operating near standstill."""

        cfg = self.config

        if self._engaged:
            if speed > cfg.release_speed:
                self._engaged = False
        else:
            if speed < cfg.engage_speed:
                self._engaged = True

        if self._longitudinal_index is not None:
            idx = self._longitudinal_index
            value = float(state[idx])
            if value < 0.0:
                state[idx] = 0.0
            elif abs(value) <= cfg.stop_speed_epsilon and not self._engaged:
                state[idx] = 0.0

        if self._yaw_rate_index is not None:
            idx = self._yaw_rate_index
            if self._engaged:
                state[idx] = 0.0
            else:
                limit = cfg.yaw_rate_limit
                state[idx] = max(-limit, min(limit, float(state[idx])))

        if self._lateral_index is not None:
            idx = self._lateral_index
            value = float(state[idx])
            if self._engaged:
                limit = cfg.stop_speed_epsilon
                state[idx] = max(-limit, min(limit, value))
            elif abs(value) <= cfg.stop_speed_epsilon:
                state[idx] = 0.0

        if self._slip_index is not None:
            idx = self._slip_index
            if self._engaged:
                state[idx] = 0.0
            else:
                limit = cfg.slip_angle_limit
                state[idx] = max(-limit, min(limit, float(state[idx])))

        if self._wheel_speed_indices:
            for idx in self._wheel_speed_indices:
                value = float(state[idx])
                if value <= 0.0:
                    state[idx] = 0.0
                elif self._engaged and value <= cfg.stop_speed_epsilon:
                    state[idx] = 0.0


__all__ = ["LowSpeedSafetyConfig", "LowSpeedSafety"]
