"""Low-speed stabilisation utilities for vehicle simulations."""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import MutableSequence, Optional, Sequence


@dataclass(frozen=True)
class LowSpeedSafetyProfile:
    engage_speed: float
    release_speed: float
    yaw_rate_limit: float
    slip_angle_limit: float

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


@dataclass(frozen=True)
class LowSpeedSafetyConfig:
    """Configuration thresholds for engaging low-speed safeguards."""

    normal: LowSpeedSafetyProfile
    drift: LowSpeedSafetyProfile
    stop_speed_epsilon: float
    drift_enabled: bool = False

    def __post_init__(self) -> None:  # type: ignore[override]
        if isinstance(self.normal, dict):
            object.__setattr__(self, "normal", LowSpeedSafetyProfile(**self.normal))
        if isinstance(self.drift, dict):
            object.__setattr__(self, "drift", LowSpeedSafetyProfile(**self.drift))
        self.normal.__class__(**asdict(self.normal))
        self.drift.__class__(**asdict(self.drift))
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
        steering_index: Optional[int] = None,
        wheelbase: Optional[float] = None,
        rear_length: Optional[float] = None,
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
        self._steering_index = steering_index
        self._wheelbase = float(wheelbase) if wheelbase is not None else None
        self._rear_length = float(rear_length) if rear_length is not None else None
        self._engaged = False
        self._drift_enabled = bool(config.drift_enabled)

    @property
    def engaged(self) -> bool:
        return self._engaged

    @property
    def drift_enabled(self) -> bool:
        return self._drift_enabled

    def set_drift_enabled(self, enabled: bool) -> None:
        self._drift_enabled = bool(enabled)

    def reset(self) -> None:
        self._engaged = False

    def apply(
        self,
        state: MutableSequence[float],
        speed: float,
        *,
        update_latch: bool = True,
    ) -> None:
        """Clamp unstable states when operating near standstill."""

        profile = self.config.drift if self._drift_enabled else self.config.normal
        stop_eps = self.config.stop_speed_epsilon

        if update_latch:
            if self._engaged:
                if speed > profile.release_speed:
                    self._engaged = False
            else:
                if speed < profile.engage_speed:
                    self._engaged = True

        yaw_target = self._kinematic_yaw_rate(state, speed)
        slip_target = self._kinematic_slip(state, speed)
        lateral_target = self._kinematic_lateral_velocity(state, speed)

        if self._engaged and speed <= stop_eps:
            beta_ref = self._velocity_slip(state)
            beta_ref = 0.0 if beta_ref is None else beta_ref
            yaw_target = 0.0
            slip_target = beta_ref
            lateral_target = speed * math.sin(beta_ref)

        if self._longitudinal_index is not None:
            idx = self._longitudinal_index
            value = float(state[idx])
            if value < 0.0:
                state[idx] = 0.0
            elif abs(value) <= stop_eps and not self._engaged:
                state[idx] = 0.0

        if self._yaw_rate_index is not None:
            idx = self._yaw_rate_index
            if self._engaged:
                limit = profile.yaw_rate_limit
                target = 0.0 if yaw_target is None else yaw_target
                state[idx] = max(-limit, min(limit, target))
            else:
                limit = profile.yaw_rate_limit
                state[idx] = max(-limit, min(limit, float(state[idx])))

        if self._lateral_index is not None:
            idx = self._lateral_index
            value = float(state[idx])
            if self._engaged:
                if lateral_target is None:
                    limit = stop_eps
                    state[idx] = max(-limit, min(limit, value))
                else:
                    state[idx] = lateral_target
            elif abs(value) <= stop_eps:
                state[idx] = 0.0

        if self._slip_index is not None:
            idx = self._slip_index
            if self._engaged:
                limit = profile.slip_angle_limit
                target = 0.0 if slip_target is None else slip_target
                state[idx] = max(-limit, min(limit, target))
            else:
                limit = profile.slip_angle_limit
                state[idx] = max(-limit, min(limit, float(state[idx])))

        if self._wheel_speed_indices:
            wheel_stage_latch = self._engaged or (
                not update_latch and speed < profile.engage_speed
            )
            for idx in self._wheel_speed_indices:
                value = float(state[idx])
                if value <= 0.0:
                    state[idx] = 0.0
                elif wheel_stage_latch and value <= stop_eps:
                    state[idx] = 0.0


    def _kinematic_beta(self, state: Sequence[float]) -> Optional[float]:
        if (
            self._steering_index is None
            or self._wheelbase is None
            or self._rear_length is None
            or self._wheelbase <= 0.0
        ):
            return None
        delta = float(state[self._steering_index])
        ratio = self._rear_length / self._wheelbase
        return math.atan(math.tan(delta) * ratio)

    def _kinematic_yaw_rate(
        self, state: Sequence[float], speed: float
    ) -> Optional[float]:
        beta = self._kinematic_beta(state)
        if beta is None or self._wheelbase is None or self._steering_index is None:
            return None
        delta = float(state[self._steering_index])
        if abs(speed) <= 1e-9:
            return 0.0
        return speed * math.cos(beta) * math.tan(delta) / self._wheelbase

    def _kinematic_slip(
        self, state: Sequence[float], _speed: float
    ) -> Optional[float]:
        return self._kinematic_beta(state)

    def _kinematic_lateral_velocity(
        self, state: Sequence[float], speed: float
    ) -> Optional[float]:
        beta = self._kinematic_beta(state)
        if beta is None:
            return None
        return speed * math.sin(beta)

    def _velocity_slip(self, state: Sequence[float]) -> Optional[float]:
        if self._longitudinal_index is None or self._lateral_index is None:
            return None
        long_idx = self._longitudinal_index
        lat_idx = self._lateral_index
        if long_idx >= len(state) or lat_idx >= len(state):
            return None

        longitudinal = float(state[long_idx])
        lateral = float(state[lat_idx])
        if abs(longitudinal) <= 1e-9 and abs(lateral) <= 1e-9:
            return 0.0
        return math.atan2(lateral, longitudinal)


__all__ = ["LowSpeedSafetyProfile", "LowSpeedSafetyConfig", "LowSpeedSafety"]
