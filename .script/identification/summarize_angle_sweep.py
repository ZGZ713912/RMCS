#!/usr/bin/env python3
"""Summarize synchronized angle sweeps for real2sim; retain raw CSV as the dataset."""
import argparse
import csv
import json
import math
from pathlib import Path

JOINTS = ("left_front", "left_back", "right_back", "right_front")

def summarize(path):
    metadata = json.loads(Path(str(path) + ".json").read_text())
    status = json.loads(Path(str(path) + ".status.json").read_text())
    result = {"metadata": metadata, "status": status, "rows": 0, "passes": {},
              "torque_is_proxy": True, "model_scope": "synchronized four-leg motion"}
    for row in csv.DictReader(path.open()):
        result["rows"] += 1
        stage = int(float(row["/identification/stage"]))
        if stage not in (3, 7, 8, 10): continue
        name = {3: "slow", 7: "fast", 8: "ultra_fast", 10: "step"}[stage]
        for joint in JOINTS:
            p = f"/chassis/{joint}_joint/"
            stats = result["passes"].setdefault(name, {}).setdefault(joint, {"samples": 0, "saturated_samples": 0})
            stats["samples"] += 1
            stats["saturated_samples"] += int(float(row[p + "command_limited"]) != 0)
            for signal in ("physical_angle", "physical_velocity", "torque", "target_torque_proxy",
                           "command_torque_proxy", "target_physical_angle", "current_raw",
                           "command_current_raw"):
                value = float(row[p + signal])
                if not math.isfinite(value): continue
                limits = stats.setdefault(signal + "_range", [value, value])
                limits[0] = min(limits[0], value); limits[1] = max(limits[1], value)
                stats.setdefault(signal + "_peak_abs", 0.0)
                stats[signal + "_peak_abs"] = max(stats[signal + "_peak_abs"], abs(value))
            if stats["samples"]:
                stats["saturated_percent"] = round(100.0 * stats["saturated_samples"] / stats["samples"], 3)
    output = path.with_suffix(".summary.json")
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(output)
    return result

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    summarize(parser.parse_args().csv)
