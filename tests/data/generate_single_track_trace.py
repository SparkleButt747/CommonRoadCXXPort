"""Generate reusable input traces for the single-track consistency tests."""

from __future__ import annotations

import csv
import math
from pathlib import Path

G = 9.81
DT = 0.01
STEPS = 100  # 1.0 seconds worth of samples [0, 0.99]


def steering_sweep(t: float) -> tuple[float, float]:
    """Return a smooth excitation for steering rate and longitudinal acceleration."""

    steering_rate = 0.12 * math.sin(2.0 * math.pi * t)
    acceleration = 0.45 * G * math.sin(1.5 * math.pi * t)
    return steering_rate, acceleration


CASES = {
    "coasting": lambda _t: (0.15, 0.0),
    "braking": lambda _t: (0.15, -0.75 * G),
    "accelerating": lambda _t: (0.15, 0.63 * G),
    "sweep": steering_sweep,
}


HEADER = ["scenario", "time", "steering_rate", "acceleration"]


def main() -> None:
    output_path = Path(__file__).with_name("single_track_trace.csv")
    with output_path.open("w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(HEADER)

        for label, command_fn in CASES.items():
            for step in range(STEPS):
                t = round(step * DT, 6)
                steering_rate, acceleration = command_fn(step * DT)
                writer.writerow([
                    label,
                    f"{t:.2f}",
                    f"{steering_rate:.8f}",
                    f"{acceleration:.8f}",
                ])

    print(f"Wrote {output_path.relative_to(Path.cwd())}")


if __name__ == "__main__":
    main()
