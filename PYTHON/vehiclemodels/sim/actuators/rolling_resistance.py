"""Rolling resistance calculations."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class RollingResistanceConfig:
    c_rr: float


class RollingResistance:
    def __init__(self, config: RollingResistanceConfig, gravity: float = 9.81) -> None:
        self.config = config
        self.gravity = gravity

    def force(self, speed: float, normal_force: float) -> float:
        if abs(speed) < 1e-3:
            return 0.0
        base = self.config.c_rr * normal_force
        return -math.copysign(base, speed)
