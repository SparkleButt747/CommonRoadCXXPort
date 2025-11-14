from __future__ import annotations

import pathlib
import sys

import pytest

PYTHON_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from vehiclemodels.sim.actuators.brake import BrakeConfig, BrakeController


def test_regen_blend_fades_near_threshold() -> None:
    config = BrakeConfig(max_force=1200.0, max_regen_force=800.0, min_regen_speed=2.0)
    controller = BrakeController(config)

    available_regen = 1000.0
    speed_below = config.min_regen_speed * 0.95
    speed_above = config.min_regen_speed * 1.05

    regen_below, hydraulic_below, total_below = controller.blend(
        brake_pedal=1.0,
        speed=speed_below,
        available_regen_force=available_regen,
    )
    regen_above, hydraulic_above, total_above = controller.blend(
        brake_pedal=1.0,
        speed=speed_above,
        available_regen_force=available_regen,
    )

    assert regen_below > 0.0
    assert regen_below < regen_above
    assert hydraulic_below > hydraulic_above
    assert total_below == pytest.approx(total_above, rel=1e-6)
    assert total_below == pytest.approx(config.max_force, rel=1e-6)
