"""Export oversteer/understeer simulation results for the single-track model.

This script mirrors PYTHON/scripts/test_vehicle.py but writes the state
trajectories of the single-track (ST) model for the three longitudinal input
cases (coasting, braking, accelerating) to a CSV file so that the C++ port can
be compared against the Python reference implementation.
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.integrate import odeint

from PYTHON.vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from PYTHON.vehiclemodels.init_st import init_st
from PYTHON.vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st


def _simulate(label: str, inputs: Iterable[float], x0: np.ndarray, p) -> np.ndarray:
    """Simulate one scenario of the ST model and return the state history."""

    t_final = 1.0
    dt = 0.01
    t = np.arange(0.0, t_final, dt)

    u = list(inputs)

    def _dyn(x, t_):
        return vehicle_dynamics_st(x, u, p)

    traj = odeint(_dyn, x0, t)
    return t, traj


def main() -> None:
    output_dir = Path("tests/output")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "oversteer_understeer_ST_python.csv"

    # load parameters and initial state (matches test_vehicle.py)
    p = parameters_vehicle2()

    g = 9.81
    initial_state = [0.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0]
    x0_st = init_st(initial_state)

    cases = [
        ("coasting", (0.15, 0.0)),
        ("braking", (0.15, -0.75 * g)),
        ("accelerating", (0.15, 0.63 * g)),
    ]

    with output_path.open("w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        header = [
            "scenario",
            "time",
            "sx",
            "sy",
            "delta",
            "velocity",
            "psi",
            "dot_psi",
            "beta",
        ]
        writer.writerow(header)

        for label, inputs in cases:
            t, traj = _simulate(label, inputs, x0_st, p)
            for time_value, state in zip(t, traj):
                writer.writerow([label, f"{time_value:.2f}", *state.tolist()])

    print(f"Wrote Python ST trajectories to {output_path}")


if __name__ == "__main__":
    main()
