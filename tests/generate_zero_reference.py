"""Generate a zero-input telemetry reference CSV for the daemon integration test."""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "PYTHON"
for path in (REPO_ROOT, PYTHON_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from PYTHON.vehiclemodels.parameters_vehicle1 import parameters_vehicle1  # type: ignore


def build_wheels(v_long: float, yaw_rate: float, track: float, steer_angle: float, axle_force: float, normal: float):
    half_track = 0.5 * max(track, 0.0)
    left_speed_body = v_long - yaw_rate * half_track
    right_speed_body = v_long + yaw_rate * half_track

    reference_speed = max(abs(v_long), 1e-6)

    left_speed = left_speed_body * math.cos(steer_angle)
    right_speed = right_speed_body * math.cos(steer_angle)

    per_wheel_force = 0.5 * axle_force
    per_wheel_normal = 0.5 * max(normal, 1e-6)

    def wheel(speed: float):
        return {
            "speed": speed,
            "slip_ratio": (speed - v_long) / reference_speed,
            "friction_utilization": abs(per_wheel_force) / per_wheel_normal,
        }

    return wheel(left_speed), wheel(right_speed)


def compute_rows(steps: int = 5, dt: float = 0.1):
    params = parameters_vehicle1()
    wheelbase = params.a + params.b

    rows = []
    cumulative_time = 0.0
    for _ in range(steps):
        state = {
            "x": 0.0,
            "y": 0.0,
            "delta": 0.0,
            "v": 0.0,
            "psi": 0.0,
            "beta": 0.0,
        }

        speed = abs(state["v"])
        heading = state["psi"] + state["beta"]
        yaw_rate = 0.0

        acceleration_long = 0.0

        front_normal = params.m * 9.81 * (params.b / wheelbase)
        rear_normal = params.m * 9.81 * (params.a / wheelbase)

        front_force = 0.0
        rear_force = 0.0

        front_left, front_right = build_wheels(speed, yaw_rate, params.T_f, state["delta"], front_force, front_normal)
        rear_left, rear_right = build_wheels(speed, yaw_rate, params.T_r, 0.0, rear_force, rear_normal)

        cumulative_time += dt
        rows.append(
            {
                "time": cumulative_time,
                "pose_x": state["x"],
                "pose_y": state["y"],
                "pose_yaw": state["psi"],
                "speed": speed,
                "v_long": state["v"],
                "v_lat": 0.0,
                "yaw_rate": yaw_rate,
                "global_x": speed * math.cos(heading),
                "global_y": speed * math.sin(heading),
                "accel_long": acceleration_long,
                "accel_lat": 0.0,
                "total_torque": 0.0,
                "drive_torque": 0.0,
                "regen_torque": 0.0,
                "mech_power": 0.0,
                "battery_power": 0.0,
                "soc": 0.7,
                "front_normal": front_normal,
                "rear_normal": rear_normal,
                "distance": 0.0,
                "energy": 0.0,
                "sim_time": cumulative_time,
                "low_speed": 1,
            }
        )
    return rows


def write_csv(rows, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        header = [
            "time",
            "pose_x",
            "pose_y",
            "pose_yaw",
            "speed",
            "v_long",
            "v_lat",
            "yaw_rate",
            "global_x",
            "global_y",
            "accel_long",
            "accel_lat",
            "total_torque",
            "drive_torque",
            "regen_torque",
            "mech_power",
            "battery_power",
            "soc",
            "front_normal",
            "rear_normal",
            "distance",
            "energy",
            "sim_time",
            "low_speed",
        ]
        writer.writerow(header)
        for row in rows:
            writer.writerow([row[key] for key in header])


def main():
    rows = compute_rows()
    output = REPO_ROOT / "tests/output/zero_input_reference.csv"
    write_csv(rows, output)
    print(f"Wrote {output} with {len(rows)} rows")


if __name__ == "__main__":
    import sys

    sys.path.insert(0, str(REPO_ROOT))
    main()
