"""Compare Python and C++ single-track simulations on shared traces."""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence

import python_scenario_runner

REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class ScenarioData:
    time: List[float]
    inputs: List[List[float]]
    states: List[List[float]]
    derivatives: List[List[float]]


TOLERANCES = {
    "inputs": 1e-12,
    "states": 5e-8,
    "derivatives": 5e-8,
}


def load_cpp_results(path: Path) -> Dict[str, ScenarioData]:
    data: Dict[str, List[List[float]]] = {}

    with path.open() as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            label = row["scenario"]
            values = [
                float(row["time"]),
                float(row["steering_rate"]),
                float(row["acceleration"]),
                float(row["sx"]),
                float(row["sy"]),
                float(row["delta"]),
                float(row["velocity"]),
                float(row["psi"]),
                float(row["dot_psi"]),
                float(row["beta"]),
                float(row["dsx"]),
                float(row["dsy"]),
                float(row["ddelta"]),
                float(row["dvelocity"]),
                float(row["dpsi"]),
                float(row["ddot_psi"]),
                float(row["dbeta"]),
            ]
            data.setdefault(label, []).append(values)

    results: Dict[str, ScenarioData] = {}
    for label, rows in data.items():
        time = [row[0] for row in rows]
        inputs = [row[1:3] for row in rows]
        states = [row[3:10] for row in rows]
        derivatives = [row[10:17] for row in rows]
        results[label] = ScenarioData(time=time, inputs=inputs, states=states, derivatives=derivatives)
    return results


def _max_abs_diff_matrix(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> float:
    max_diff = 0.0
    for row_a, row_b in zip(a, b):
        for val_a, val_b in zip(row_a, row_b):
            diff = abs(val_a - val_b)
            if diff > max_diff:
                max_diff = diff
    return max_diff


def compare_scenarios(python_results: Dict[str, python_scenario_runner.Trajectory],
                      cpp_results: Dict[str, ScenarioData]) -> None:
    missing = set(python_results) ^ set(cpp_results)
    if missing:
        raise AssertionError(f"mismatched scenarios: {missing}")

    for label in sorted(python_results.keys()):
        py = python_results[label]
        cpp = cpp_results[label]

        for t_py, t_cpp in zip(py.time, cpp.time):
            if not math.isclose(t_py, t_cpp, abs_tol=1e-12):
                raise AssertionError(f"time mismatch for {label}: {t_py} vs {t_cpp}")

        diffs = {
            "inputs": _max_abs_diff_matrix(cpp.inputs, py.inputs),
            "states": _max_abs_diff_matrix(cpp.states, py.states),
            "derivatives": _max_abs_diff_matrix(cpp.derivatives, py.derivatives),
        }

        for key, value in diffs.items():
            if value > TOLERANCES[key]:
                raise AssertionError(
                    f"{label}: {key} difference {value:.3e} exceeds tolerance {TOLERANCES[key]:.3e}"
                )

        print(
            f"{label:>12} | inputs: {diffs['inputs']:.3e} | "
            f"states: {diffs['states']:.3e} | derivatives: {diffs['derivatives']:.3e}"
        )


def run_cpp_simulator(build_dir: Path, input_path: Path, output_path: Path) -> None:
    exe = build_dir / "scenario_simulator"
    if sys.platform == "win32":
        exe = exe.with_suffix(".exe")

    if not exe.exists():
        raise FileNotFoundError(f"scenario_simulator binary not found at {exe}")

    cmd = [str(exe), str(input_path), str(output_path)]
    subprocess.run(cmd, check=True)


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, required=True, help="CMake build directory")
    parser.add_argument(
        "--input", type=Path, default=REPO_ROOT / "tests/data/single_track_trace.csv", help="Input trace CSV"
    )
    parser.add_argument(
        "--cpp-output", type=Path, default=REPO_ROOT / "tests/output/single_track_trace_cpp.csv",
        help="Output CSV for the C++ simulator",
    )

    args = parser.parse_args(argv)

    python_results = python_scenario_runner.simulate_all(args.input)
    python_output = REPO_ROOT / "tests/output/single_track_trace_python.csv"
    python_scenario_runner.write_csv(python_results, python_output)

    run_cpp_simulator(args.build_dir, args.input, args.cpp_output)

    cpp_results = load_cpp_results(args.cpp_output)
    compare_scenarios(python_results, cpp_results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
