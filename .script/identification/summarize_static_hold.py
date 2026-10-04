#!/usr/bin/env python3
"""Summarize dedicated static holds; torque columns are uncalibrated proxies."""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

JOINTS = ("left_front", "left_back", "right_back", "right_front")


def summarize(path: Path, output: Path) -> list[dict]:
    metadata = json.loads(Path(str(path) + ".json").read_text())
    status = json.loads(Path(str(path) + ".status.json").read_text())
    if status.get("write_failed") is not False or status.get("dropped_samples") != 0:
        raise ValueError("Incomplete timing: recorder failure or lost samples")
    groups = defaultdict(list)
    seen = set()
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if float(row["/identification/stage"]) != 7 or float(row["/identification/actuation_active"]) != 1:
                continue
            point = int(float(row["/identification/point"]))
            direction = int(float(row["/identification/approach_direction"]))
            host = float(row["/identification/host_time_s"])
            for joint in JOINTS:
                p = f"/chassis/{joint}_joint/"
                sequence = float(row[p + "feedback_sequence"])
                key = (point, joint, sequence)
                if key in seen:
                    continue
                seen.add(key)
                q, v, target, current, torque, physical, temp, time = (
                    float(row[p + s]) for s in ("physical_angle", "physical_velocity",
                    "target_physical_angle", "current_raw", "torque", "physical_torque_proxy",
                    "temperature", "feedback_time_s"))
                values = [q, v, target, current, torque, physical, temp, time]
                if not np.all(np.isfinite(values)) or not 0 <= host - time <= metadata["max_feedback_age_s"]:
                    continue
                if abs(q - target) > metadata["angle_tolerance_rad"] or abs(v) > metadata["settle_velocity_rad_s"]:
                    continue
                if any(float(row[p + s]) != 0 for s in ("command_limited", "current_clipped", "current_rate_limited")):
                    continue
                if p + "command_current_raw" in row and abs(float(row[p + "command_current_raw"])) >= metadata["max_command_current_raw"]:
                    continue
                groups[(joint, point, direction)].append(values)
    results = []
    for (joint, point, direction), values in sorted(groups.items()):
        a = np.asarray(values)
        result = {"condition": metadata["condition"], "joint": joint, "point": point,
                  "approach_direction": direction, "target_angle_deg": float(np.rad2deg(np.median(a[:, 2]))),
                  "samples": len(a), "duration_s": float(np.ptp(a[:, 7])),
                  "run_completed": status["completed"], "torque_calibrated": False}
        result["hold_validated"] = bool(status["completed"] and len(a) >= 20
            and np.ptp(a[:, 7]) >= 0.9 * metadata["hold_duration_s"])
        for column, name in ((0, "angle_rad"), (3, "current_raw"), (4, "motor_torque_proxy_Nm"),
                             (5, "physical_torque_proxy_Nm"), (6, "temperature_c")):
            for statistic, fn in (("mean", np.mean), ("median", np.median), ("std", np.std)):
                result[name + "_" + statistic] = float(fn(a[:, column]))
        results.append(result)
    if not results:
        raise ValueError("No valid dedicated static hold samples")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    model = {"condition": metadata["condition"], "torque_calibrated": False,
             "coordinate": "physical leg angle; torque uses same axis",
             "interpolation": "linear within measured angle range; do not extrapolate",
             "points": results}
    output.with_suffix(".json").write_text(json.dumps(model, indent=2, allow_nan=False) + "\n")
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(f"Static groups: {len(summarize(args.csv, args.output))}; output: {args.output}")
