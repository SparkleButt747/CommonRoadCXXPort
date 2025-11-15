"""Simulate single-track scenarios using the Python reference implementation."""

from __future__ import annotations

import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
PYTHON_ROOT = REPO_ROOT / "PYTHON"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from PYTHON.vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from PYTHON.vehiclemodels.init_st import init_st
from PYTHON.vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st


@dataclass
class Sample:
    time: float
    steering_rate: float
    acceleration: float


TraceMap = Dict[str, List[Sample]]


def load_trace(path: Path) -> TraceMap:
    traces: TraceMap = {}
    with path.open() as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            sample = Sample(
                time=float(row["time"]),
                steering_rate=float(row["steering_rate"]),
                acceleration=float(row["acceleration"]),
            )
            traces.setdefault(row["scenario"], []).append(sample)

    if not traces:
        raise ValueError(f"no samples in trace: {path}")
    return traces


def rk4_step(x: List[float], u: List[float], params, dt: float) -> List[float]:
    def dyn(state: List[float]) -> List[float]:
        return [float(v) for v in vehicle_dynamics_st(state, u, params)]

    k1 = dyn(x)
    x_tmp = [xi + 0.5 * dt * ki for xi, ki in zip(x, k1)]
    k2 = dyn(x_tmp)
    x_tmp = [xi + 0.5 * dt * ki for xi, ki in zip(x, k2)]
    k3 = dyn(x_tmp)
    x_tmp = [xi + dt * ki for xi, ki in zip(x, k3)]
    k4 = dyn(x_tmp)

    return [
        xi + (dt / 6.0) * (k1i + 2.0 * k2i + 2.0 * k3i + k4i)
        for xi, k1i, k2i, k3i, k4i in zip(x, k1, k2, k3, k4)
    ]


@dataclass
class Trajectory:
    label: str
    time: List[float]
    inputs: List[List[float]]
    states: List[List[float]]
    derivatives: List[List[float]]


def simulate(label: str, samples: Iterable[Sample], x0: List[float], params) -> Trajectory:
    samples = list(samples)
    if not samples:
        raise ValueError(f"scenario '{label}' has no samples")

    times = [sample.time for sample in samples]
    inputs = [[sample.steering_rate, sample.acceleration] for sample in samples]

    states: List[List[float]] = []
    derivatives: List[List[float]] = []

    x = list(x0)
    for idx, sample in enumerate(samples):
        states.append(list(x))
        derivatives.append([float(v) for v in vehicle_dynamics_st(x, inputs[idx], params)])

        if idx + 1 < len(samples):
            dt = samples[idx + 1].time - sample.time
        elif idx > 0:
            dt = sample.time - samples[idx - 1].time
        else:
            raise ValueError("need at least two samples to determine dt")

        if dt <= 0:
            raise ValueError("non-positive dt encountered")

        x = rk4_step(x, inputs[idx], params, dt)

    return Trajectory(label=label, time=times, inputs=inputs, states=states, derivatives=derivatives)


def simulate_all(trace_path: Path) -> Dict[str, Trajectory]:
    params = parameters_vehicle2()
    initial_state = [0.0, 0.0, 0.0, 15.0, 0.0, 0.0, 0.0]
    x0 = list(init_st(initial_state))

    traces = load_trace(trace_path)
    trajectories: Dict[str, Trajectory] = {}
    for label, samples in traces.items():
        trajectories[label] = simulate(label, samples, x0, params)
    return trajectories


def write_csv(trajectories: Dict[str, Trajectory], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(
            [
                "scenario",
                "time",
                "steering_rate",
                "acceleration",
                "sx",
                "sy",
                "delta",
                "velocity",
                "psi",
                "dot_psi",
                "beta",
                "dsx",
                "dsy",
                "ddelta",
                "dvelocity",
                "dpsi",
                "ddot_psi",
                "dbeta",
            ]
        )

        for label in sorted(trajectories.keys()):
            traj = trajectories[label]
            for t, u, state, deriv in zip(traj.time, traj.inputs, traj.states, traj.derivatives):
                writer.writerow([
                    label,
                    f"{t:.2f}",
                    f"{u[0]:.10f}",
                    f"{u[1]:.10f}",
                    *[f"{value:.10f}" for value in state],
                    *[f"{value:.10f}" for value in deriv],
                ])


def main() -> None:
    trace_path = REPO_ROOT / "tests/data/single_track_trace.csv"
    output_path = REPO_ROOT / "tests/output/single_track_trace_python.csv"

    trajectories = simulate_all(trace_path)
    write_csv(trajectories, output_path)
    print(f"python_scenario_runner: wrote {output_path}")


if __name__ == "__main__":
    main()
