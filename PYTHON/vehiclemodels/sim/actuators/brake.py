"""Brake controller blending hydraulic and regenerative braking."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class BrakeConfig:
    max_force: float
    max_regen_force: float
    min_regen_speed: float


class BrakeController:
    def __init__(self, config: BrakeConfig) -> None:
        self.config = config

    def blend(self, brake_pedal: float, speed: float, available_regen_force: float) -> tuple[float, float, float]:
        brake_pedal = min(max(brake_pedal, 0.0), 1.0)
        total_force = min(self.config.max_force, brake_pedal * self.config.max_force)

        if abs(speed) < self.config.min_regen_speed:
            regen_force = 0.0
        else:
            regen_force = min(total_force, brake_pedal * self.config.max_regen_force, available_regen_force)

        hydraulic_force = max(0.0, total_force - regen_force)
        return regen_force, hydraulic_force, total_force
