#!/usr/bin/env python3
"""Fit count-domain actuator/plant response; never interpret raw iq as measured Nm."""

from __future__ import annotations

import argparse
from array import array
import csv
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter


JOINTS = ("left_front", "left_back", "right_back", "right_front")
FEATURES = ("current_raw", "current_raw*angle_offset", "current_raw*abs_velocity",
            "velocity", "tanh_10_velocity", "sin_angle", "cos_angle", "bias")
SAMPLE_FIELDS = ("time_s", "angle", "velocity", "current_raw", "temperature", "point",
                 "stage", "amplitude_raw", "frequency_hz", "probe_raw", "valid")


@dataclass
class Dataset:
    metadata: dict
    status: dict
    samples: list[np.ndarray]
    commands: list[np.ndarray]
    quality: list[dict]


def load_dataset(path: Path, allow_partial: bool = False) -> Dataset:
    metadata = json.loads(Path(str(path) + ".json").read_text())
    status = json.loads(Path(str(path) + ".status.json").read_text())
    if metadata.get("command_domain") != "raw_current" or metadata.get("schema_version") != 1:
        raise ValueError("Require schema 1 raw_current metadata; torque CSV is a different experiment")
    if status.get("write_failed") is not False or status.get("dropped_samples") != 0:
        raise ValueError("Recorder write failure or queue loss invalidates timing")
    if status.get("completed") is not True and not allow_partial:
        raise ValueError("Sweep did not complete; --allow-partial enables diagnostic fits only")
    prefixes = [f"/chassis/{j}_joint" for j in JOINTS]
    if metadata.get("targets") != prefixes:
        raise ValueError("Unexpected joint order or physical-axis contract")
    samples = [array("d") for _ in JOINTS]
    commands = [array("d") for _ in JOINTS]
    last_command = np.zeros(4)
    last_feedback = np.zeros(4)
    quality = [dict(eligible_rows=0, duplicate_feedback=0, rejected_samples=0) for _ in JOINTS]
    previous_host = -np.inf
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        required = {"schema_version", "host_time_s", "stage", "point_index", "active_joint",
                    "actuation_active", "amplitude_raw", "frequency_hz", "queue_dropped"}
        suffixes = ("physical_angle", "physical_velocity", "current_raw", "temperature",
                    "feedback_sequence", "feedback_time_s", "command_sequence",
                    "command_prepare_time_s", "command_current_raw", "command_limited",
                    "current_clipped", "current_rate_limited", "sweep_probe_current_raw")
        required.update(f"{p}/{s}" for p in prefixes for s in suffixes)
        if not reader.fieldnames or required.difference(reader.fieldnames):
            raise ValueError("Missing current-sweep columns: " + str(required.difference(reader.fieldnames or [])))
        for row in reader:
            host = float(row["host_time_s"])
            if not np.isfinite(host) or host <= previous_host or float(row["schema_version"]) != 1:
                raise ValueError("Nonmonotonic host timestamps or mixed CSV schema")
            previous_host = host
            if float(row["queue_dropped"]) != 0:
                raise ValueError("CSV reports dropped rows")
            # Packet preparation timestamps, never scheduled executor timestamps.
            for j, p in enumerate(prefixes):
                sequence = float(row[p + "/command_sequence"])
                if not np.isfinite(sequence) or sequence < 0 or sequence != int(sequence):
                    raise ValueError("Invalid command sequence")
                if sequence < last_command[j]:
                    raise ValueError("Command sequence reset; do not concatenate separate runs")
                if sequence == last_command[j]:
                    continue
                last_command[j] = sequence
                t = float(row[p + "/command_prepare_time_s"])
                u = float(row[p + "/command_current_raw"])
                if np.isfinite(t) and np.isfinite(u):
                    if t > host or (commands[j] and t <= commands[j][-2]):
                        raise ValueError("Invalid command preparation time")
                    commands[j].extend((t, u))
            stage = float(row["stage"])
            if stage not in (3, 4):
                continue
            j = int(float(row["active_joint"]))
            if j not in range(4):
                raise ValueError("Invalid active joint")
            p = prefixes[j]
            quality[j]["eligible_rows"] += 1
            sequence = float(row[p + "/feedback_sequence"])
            if not np.isfinite(sequence) or sequence < last_feedback[j]:
                raise ValueError("Invalid/reset feedback sequence")
            if sequence == last_feedback[j]:
                quality[j]["duplicate_feedback"] += 1
                continue
            last_feedback[j] = sequence
            t = float(row[p + "/feedback_time_s"])
            q, v, iq, temp = (float(row[p + "/" + s]) for s in
                               ("physical_angle", "physical_velocity", "current_raw", "temperature"))
            # Conservative rejection includes other joints reaching final count saturation.
            clipped = any(float(row[k + "/" + s]) != 0 for k in prefixes for s in
                          ("command_limited", "current_clipped", "current_rate_limited"))
            clipped |= any(abs(float(row[k + "/command_current_raw"])) >=
                           float(metadata["max_command_current_raw"]) for k in prefixes)
            valid = (np.all(np.isfinite([t, q, v, iq, temp])) and sequence >= 1
                     and 0 <= host - t <= float(metadata.get("max_feedback_age_s", 0.05))
                     and float(row["actuation_active"]) == 1 and not clipped)
            quality[j]["rejected_samples"] += int(not valid)
            samples[j].extend((t, q, v, iq, temp, float(row["point_index"]), stage,
                               float(row["amplitude_raw"]), float(row["frequency_hz"]),
                               float(row[p + "/sweep_probe_current_raw"]), float(valid)))
    return Dataset(metadata, status,
                   [np.array(a).reshape(-1, len(SAMPLE_FIELDS)) for a in samples],
                   [np.array(a).reshape(-1, 2) for a in commands], quality)


def blocks(samples: np.ndarray, window: int) -> list[np.ndarray]:
    if window < 5 or window % 2 != 1:
        raise ValueError("Smoothing window must be odd and at least 5 feedback samples")
    if len(samples) < window * 2:
        raise ValueError("Insufficient distinct feedback samples")
    dt = np.diff(samples[:, 0])
    if np.any(~np.isfinite(dt)) or np.any(dt <= 0):
        raise ValueError("Distinct feedback times must be strictly increasing")
    nominal = np.median(dt[dt < 0.02])
    if not np.isfinite(nominal) or nominal <= 0:
        raise ValueError("Cannot infer feedback sample period")
    breaks = ((dt > max(0.01, 4 * nominal))
              | np.any(np.diff(samples[:, [5, 6, 7]], axis=0) != 0, axis=1)
              | (samples[:-1, -1] == 0) | (samples[1:, -1] == 0))
    result = []
    for part in np.split(samples, np.flatnonzero(breaks) + 1):
        if len(part) < max(40, 2 * window) or np.any(part[:, -1] == 0):
            continue
        h = float(np.median(np.diff(part[:, 0])))
        grid = np.arange(part[0, 0], part[-1, 0], h)
        if len(grid) < 2 * window:
            continue
        out = np.column_stack([grid] + [np.interp(grid, part[:, 0], part[:, i])
                                       for i in range(1, len(SAMPLE_FIELDS))])
        v = savgol_filter(out[:, 2], window, 3)
        acceleration = savgol_filter(out[:, 2], window, 3, deriv=1, delta=h)
        out[:, 2] = v
        out = np.column_stack((out, acceleration))
        # No derivative/filter windows may bridge stage transitions or rejected data.
        half = window // 2
        result.append(out[half:-half])
    if not result:
        raise ValueError("No contiguous unsaturated excitation blocks remain")
    return result


def features(q: np.ndarray, v: np.ndarray, iq: np.ndarray, center: float) -> np.ndarray:
    return np.column_stack((iq, iq * (q - center), iq * np.abs(v), v,
                            np.tanh(10 * v), np.sin(q), np.cos(q), np.ones_like(q)))


def metrics(actual: np.ndarray, prediction: np.ndarray) -> dict:
    residual = prediction - actual
    denominator = float(np.sum((actual - np.mean(actual)) ** 2))
    return {"rmse": float(np.sqrt(np.mean(residual ** 2))),
            "r2": 1 - float(np.sum(residual ** 2)) / denominator if denominator > 1e-12 else None}


def command_at(commands: np.ndarray, time_s: np.ndarray) -> np.ndarray:
    index = np.searchsorted(commands[:, 0], time_s, side="right") - 1
    if np.any(index < 0):
        raise ValueError("Feedback lacks preceding prepared commands")
    return commands[index, 1]


def plant_rollout(segments: list[np.ndarray], coefficient: np.ndarray, center: float) -> dict:
    observed, predicted = [], []
    velocity_observed, velocity_predicted = [], []
    for b in segments:
        h = float(np.median(np.diff(b[:, 0])))
        length = max(20, int(0.5 / h))
        for begin in range(0, len(b) - length, length):
            part = b[begin:begin + length + 1]
            q, v = part[0, 1:3]
            for k in range(1, len(part)):
                dt = part[k, 0] - part[k - 1, 0]
                iq = part[k - 1, 3]
                f = [iq, iq * (q - center), iq * abs(v), v, np.tanh(10 * v), np.sin(q), np.cos(q), 1]
                acceleration = float(np.dot(f, coefficient))
                q += v * dt + 0.5 * acceleration * dt * dt
                v += acceleration * dt
                if not np.isfinite(q) or not np.isfinite(v) or abs(v) > 100:
                    return {"horizon_s": 0.5, "stable": False}
                observed.append(part[k, 1])
                predicted.append(q)
                velocity_observed.append(part[k, 2])
                velocity_predicted.append(v)
    if not observed:
        return {"horizon_s": 0.5, "stable": False}
    return {"horizon_s": 0.5, "stable": True,
            "angle": metrics(np.array(observed), np.array(predicted)),
            "velocity": metrics(np.array(velocity_observed), np.array(velocity_predicted))}


def fit_transport(train: list[np.ndarray], validation: list[np.ndarray], commands: np.ndarray,
                  max_delay: float) -> dict:
    # First-order iq response + effective host latency. Fit only chirp segments.
    t0 = np.concatenate([b[:-1, 0] for b in train])
    t1 = np.concatenate([b[1:, 0] for b in train])
    i0 = np.concatenate([b[:-1, 3] for b in train])
    i1 = np.concatenate([b[1:, 3] for b in train])
    indices = np.linspace(0, len(t0) - 1, min(len(t0), 12000), dtype=int)
    t0, t1, i0, i1 = (a[indices] for a in (t0, t1, i0, i1))
    best = None
    taus = np.r_[0.0, np.geomspace(0.0005, 0.12, 36)]
    for delay in np.linspace(0, max_delay, 41):
        u = command_at(commands, (t0 + t1) / 2 - delay)
        for tau in taus:
            a = np.exp(-(t1 - t0) / tau) if tau > 0 else np.zeros_like(t0)
            w = 1 - a
            x = np.column_stack((w * u, w))
            coefficient, _, rank, _ = np.linalg.lstsq(x, i1 - a * i0, rcond=None)
            if rank < 2 or not 0 < coefficient[0] < 8:
                continue
            error = float(np.mean((a * i0 + x @ coefficient - i1) ** 2))
            if best is None or error < best[0]:
                best = (error, float(delay), float(tau), coefficient)
    if best is None:
        raise ValueError("Unidentifiable current response")
    _, delay, tau, coefficient = best
    observed, predicted = [], []
    for b in validation:
        u = command_at(commands, (b[:-1, 0] + b[1:, 0]) / 2 - delay)
        h = np.diff(b[:, 0])
        prediction = np.empty(len(b))
        prediction[0] = b[0, 3]
        for k in range(1, len(b)):
            a = np.exp(-h[k - 1] / tau) if tau > 0 else 0.0
            prediction[k] = a * prediction[k - 1] + (1 - a) * (coefficient[0] * u[k - 1] + coefficient[1])
        observed.extend(b[1:, 3])
        predicted.extend(prediction[1:])
    return {"delay_s": delay, "time_constant_s": tau, "gain": float(coefficient[0]),
            "bias_raw": float(coefficient[1]), "validation": metrics(np.array(observed), np.array(predicted)),
            "search_boundary": bool(delay >= max_delay or tau >= taus[-1]),
            "timestamp_scope": "host prepare to host receive, includes transport/feedback latency"}


def fit_joint(dataset: Dataset, joint: int, window: int = 11, max_delay: float = 0.04) -> tuple[dict, dict]:
    segments = blocks(dataset.samples[joint], window)
    train = [b for b in segments if b[0, 6] == 3]
    validation = [b for b in segments if b[0, 6] == 4]
    if not train or not validation:
        raise ValueError("Require chirp training and separate held-out step blocks")
    a, v = np.concatenate(train), np.concatenate(validation)
    span = float(np.ptp(a[:, 1]))
    if span < 0.25 or np.std(a[:, 3]) < 16 or np.ptp(a[:, 2]) < 0.02:
        raise ValueError("Insufficient angle/current/velocity excitation; collect more states")
    if not np.any(a[:, 2] < -0.01) or not np.any(a[:, 2] > 0.01):
        raise ValueError("Require both physical directions")
    if not np.any(a[:, 9] < -16) or not np.any(a[:, 9] > 16):
        raise ValueError("Missing independent probe excitation")
    center = float(np.mean(a[:, 1]))
    x = features(a[:, 1], a[:, 2], a[:, 3], center)
    scale = np.linalg.norm(x, axis=0)
    if np.any(scale < 1e-12):
        raise ValueError("Unexcited dynamics features")
    normalized = x / scale
    condition = float(np.linalg.cond(normalized))
    if not np.isfinite(condition) or condition > 1e7 or np.linalg.matrix_rank(normalized) < len(FEATURES):
        raise ValueError("Dynamics model is not identifiable for these states")
    coefficient = np.linalg.lstsq(normalized, a[:, -1], rcond=None)[0] / scale
    train_score = metrics(a[:, -1], x @ coefficient)
    validation_score = metrics(v[:, -1], features(v[:, 1], v[:, 2], v[:, 3], center) @ coefficient)
    rollout = plant_rollout(validation, coefficient, center)
    transport = fit_transport(train, validation, dataset.commands[joint], max_delay)
    # Sign is in the physical leg-angle axis, independent of CAN reversal configuration.
    direction = float(dataset.metadata["current_to_physical_direction"][joint])
    sign_ok = np.all(direction * (coefficient[0] + coefficient[1] * (a[:, 1] - center)
                                 + coefficient[2] * np.abs(a[:, 2])) > 0)
    validated = (dataset.status["completed"] and sign_ok
                 and validation_score["r2"] is not None and validation_score["r2"] >= 0.7
                 and transport["validation"]["r2"] is not None
                 and transport["validation"]["r2"] >= 0.7 and not transport["search_boundary"]
                 and rollout["stable"] and rollout["angle"]["rmse"] < 0.03
                 and rollout["velocity"]["r2"] is not None and rollout["velocity"]["r2"] >= 0.7)
    coverage = {"angle_rad": [float(np.min(a[:, 1])), float(np.max(a[:, 1]))],
                "velocity_rad_s": [float(np.min(a[:, 2])), float(np.max(a[:, 2]))],
                "feedback_current_raw": [float(np.min(a[:, 3])), float(np.max(a[:, 3]))],
                "temperature_c": [float(np.min(a[:, 4])), float(np.max(a[:, 4]))],
                "amplitudes_raw": np.unique(a[:, 7]).tolist(),
                "frequency_hz": [float(np.min(a[:, 8])), float(np.max(a[:, 8]))]}
    model = {"joint": JOINTS[joint], "condition": dataset.metadata["condition"],
             "training_ready": bool(validated), "absolute_torque_identified": False,
             "feature_names": list(FEATURES), "coefficients": coefficient.tolist(),
             "angle_center_rad": center, "output_unit": "rad/s^2",
             "interpretation": "effective joint acceleration from measured iq counts and state; inertia/Kt/contact not separated",
             "estimator": "closed-loop least squares; derivative/noise bias requires held-out validation",
             "design_condition": condition, "train": train_score, "validation": validation_score,
             "validation_rollout": rollout,
             "transport": transport, "coverage": coverage,
             "train_samples": len(a), "validation_samples": len(v),
             "feedback_period_s": float(np.median(np.concatenate([np.diff(b[:, 0]) for b in segments]))),
             "quality": dataset.quality[joint]}
    # Features/labels describe motion, never fabricated torque labels.
    pack = {"train": a, "validation": v}
    return model, pack


def fit_file(csv_path: Path, output: Path, joints: list[str], allow_partial: bool = False,
             window: int = 11, max_delay: float = 0.04) -> dict:
    if not np.isfinite(max_delay) or max_delay <= 0:
        raise ValueError("max_delay must be positive")
    data = load_dataset(csv_path, allow_partial)
    models, packs, failures = {}, {}, {}
    for name in joints:
        j = JOINTS.index(name)
        try:
            models[name], pack = fit_joint(data, j, window, max_delay)
            packs.update({f"{name}_{split}": values for split, values in pack.items()})
        except ValueError as e:
            failures[name] = str(e)
    result = {"schema_version": 1, "command_domain": "raw_current",
              "source_csv": str(csv_path.resolve()), "condition": data.metadata["condition"],
              "source_metadata": data.metadata, "source_status": data.status,
              "dataset_columns": list(SAMPLE_FIELDS) + ["acceleration_rad_s2"],
              "training_ready": len(models) == len(joints) and all(m["training_ready"] for m in models.values()),
              "models": models, "failures": failures}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    np.savez_compressed(output.with_suffix(".npz"), **packs,
                        columns=np.array(result["dataset_columns"]))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--joint", choices=JOINTS, action="append")
    parser.add_argument("--allow-partial", action="store_true", help="Diagnostic only; never training_ready")
    parser.add_argument("--window", type=int, default=11)
    parser.add_argument("--max-delay-s", type=float, default=0.04)
    args = parser.parse_args()
    result = fit_file(args.csv, args.output, args.joint or list(JOINTS),
                      args.allow_partial, args.window, args.max_delay_s)
    print(json.dumps({"training_ready": result["training_ready"], "failures": result["failures"],
                      "output": str(args.output)}, ensure_ascii=False))
    return 0 if result["training_ready"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
