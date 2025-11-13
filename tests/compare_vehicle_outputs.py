"""Compare Python and C++ trajectories for the ST oversteer/understeer case."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

CSVRow = Tuple[float, List[float]]


def _load(path: Path) -> Dict[str, List[CSVRow]]:
    data: Dict[str, List[CSVRow]] = {}
    with path.open() as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            scenario = row["scenario"]
            time = float(row["time"])
            state = [
                float(row["sx"]),
                float(row["sy"]),
                float(row["delta"]),
                float(row["velocity"]),
                float(row["psi"]),
                float(row["dot_psi"]),
                float(row["beta"]),
            ]
            data.setdefault(scenario, []).append((time, state))
    return data


def _align(py_rows: List[CSVRow], cpp_rows: List[CSVRow]) -> Tuple[np.ndarray, np.ndarray]:
    if len(py_rows) != len(cpp_rows):
        raise ValueError("Mismatched trajectory length")
    py = np.array([state for _, state in py_rows])
    cpp = np.array([state for _, state in cpp_rows])
    return py, cpp


def main() -> None:
    py_path = Path("tests/output/oversteer_understeer_ST_python.csv")
    cpp_path = Path("tests/output/oversteer_understeer_ST_cpp.csv")

    if not py_path.exists() or not cpp_path.exists():
        raise SystemExit("Missing CSV outputs; run the generators first.")

    py_data = _load(py_path)
    cpp_data = _load(cpp_path)

    keys = sorted(py_data.keys())
    header = (
        f"{'scenario':>12} | {'state':>8} | {'max|Δ|':>12} | {'RMSE':>12}"
    )
    print(header)
    print("-" * len(header))

    state_names = ["sx", "sy", "delta", "velocity", "psi", "dot_psi", "beta"]

    for key in keys:
        py_rows = py_data[key]
        cpp_rows = cpp_data.get(key)
        if cpp_rows is None:
            print(f"{key:>12} | {'missing':>8} | {'-':>12} | {'-':>12}")
            continue

        py_states, cpp_states = _align(py_rows, cpp_rows)
        diff = cpp_states - py_states
        max_abs = np.max(np.abs(diff), axis=0)
        rmse = np.sqrt(np.mean(diff ** 2, axis=0))

        for idx, name in enumerate(state_names):
            prefix = key if idx == 0 else ""
            print(
                f"{prefix:>12} | {name:>8} | "
                f"{max_abs[idx]:12.6e} | {rmse[idx]:12.6e}"
            )

        print("-" * len(header))


if __name__ == "__main__":
    main()
