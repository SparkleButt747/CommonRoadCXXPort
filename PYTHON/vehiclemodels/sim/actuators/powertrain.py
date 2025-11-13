"""Simple longitudinal powertrain model with SOC tracking."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class PowertrainConfig:
    max_drive_torque: float
    max_regen_torque: float
    max_power: float
    drive_efficiency: float
    regen_efficiency: float
    min_soc: float
    max_soc: float
    initial_soc: float
    battery_capacity_kwh: float


class Powertrain:
    """Maps pedal demand and vehicle speed to wheel torque."""

    def __init__(self, config: PowertrainConfig, wheel_radius: float) -> None:
        if wheel_radius <= 0:
            raise ValueError("wheel_radius must be positive")
        self.config = config
        self.wheel_radius = wheel_radius
        self.capacity_j = config.battery_capacity_kwh * 3.6e6
        if self.capacity_j <= 0:
            raise ValueError("battery capacity must be positive")
        self.soc = float(config.initial_soc)

    # ------------------------------------------------------------------
    def _torque_power_limited(self, speed: float) -> float:
        if self.config.max_power <= 0:
            return math.inf
        wheel_speed = abs(speed) / self.wheel_radius
        if wheel_speed < 1e-6:
            return self.config.max_drive_torque
        return self.config.max_power / wheel_speed

    def available_drive_torque(self, speed: float) -> float:
        if self.soc <= self.config.min_soc:
            return 0.0
        torque = min(self.config.max_drive_torque, self._torque_power_limited(speed))
        return max(0.0, torque)

    def available_regen_torque(self, speed: float) -> float:
        if self.soc >= self.config.max_soc:
            return 0.0
        if abs(speed) < 1e-3:
            return 0.0
        torque = min(self.config.max_regen_torque, self._torque_power_limited(speed))
        return max(0.0, torque)

    # ------------------------------------------------------------------
    def step(
        self, throttle: float, regen_torque_request: float, speed: float, dt: float
    ) -> tuple[float, float, float]:
        """Return total, drive and regen torque at the wheel."""

        throttle = min(max(throttle, 0.0), 1.0)
        regen_torque_request = max(0.0, regen_torque_request)

        drive_torque = min(throttle * self.config.max_drive_torque, self.available_drive_torque(speed))
        regen_torque = min(regen_torque_request, self.available_regen_torque(speed))

        wheel_speed = speed / self.wheel_radius
        mechanical_drive_power = drive_torque * wheel_speed
        mechanical_regen_power = -regen_torque * wheel_speed

        battery_power = 0.0
        if mechanical_drive_power > 0:
            battery_power += mechanical_drive_power / max(self.config.drive_efficiency, 1e-6)
        else:
            mechanical_drive_power = 0.0
        if mechanical_regen_power < 0:
            battery_power += mechanical_regen_power * self.config.regen_efficiency
        else:
            mechanical_regen_power = 0.0

        soc_delta = -battery_power * dt / self.capacity_j if dt > 0 else 0.0
        self.soc = min(self.config.max_soc, max(self.config.min_soc, self.soc + soc_delta))

        total_torque = drive_torque - regen_torque
        return total_torque, drive_torque, regen_torque
