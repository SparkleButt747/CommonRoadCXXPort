"""Aerodynamic drag and downforce helpers."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class AeroConfig:
    drag_coefficient: float
    downforce_coefficient: float


class AeroModel:
    def __init__(self, config: AeroConfig) -> None:
        self.config = config

    def drag_force(self, speed: float) -> float:
        coeff = self.config.drag_coefficient
        return -math.copysign(coeff * speed * speed, speed) if speed else 0.0

    def downforce(self, speed: float) -> float:
        coeff = self.config.downforce_coefficient
        if coeff == 0:
            return 0.0
        return abs(coeff) * speed * speed
