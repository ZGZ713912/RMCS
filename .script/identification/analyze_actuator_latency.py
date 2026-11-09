#!/usr/bin/env python3
"""Measure host timing and identify command-to-reported-current effective dynamics."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.linalg import solve_banded
from scipy.optimize import least_squares
from scipy.signal import csd, welch

AXES = ("left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint", "left_wheel", "right_wheel")
DTYPE = [("event", "i4"), ("axis", "i4"), ("sequence", "u8"), ("steady_ns", "u8"),
         ("end_steady_ns", "u8"), ("value", "f8"), ("code", "i4")]


def read_events(path):
    records = []
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        for line, row in enumerate(reader, 2):
            if None in row or any(row.get(key) is None for key, _ in DTYPE):
                raise ValueError(f"Incomplete event at line {line}")
            records.append(tuple(float(row[key]) if key == "value" else int(row[key]) for key, _ in DTYPE))
    if not records:
        raise ValueError("Empty latency trace")
    return np.array(records, dtype=DTYPE)


def summary(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(zip(("count", "min", "p50", "p95", "p99", "max"),
                    (len(values), float(np.min(values)), *map(float, np.percentile(values, [50, 95, 99])), float(np.max(values)))))


def milliseconds(end, start):
    return (np.asarray(end).astype(np.int64) - np.asarray(start).astype(np.int64)) * 1e-6


def metrics(actual, prediction):
    rmse = float(np.sqrt(np.mean((actual - prediction) ** 2)))
    return {"rmse_nm": rmse, "nrmse": rmse / max(float(np.std(actual)), 1e-12)}


def response(command_t, command, query_t, delay_ms, lag_ms):
    times = command_t + delay_ms * .001
    indices = np.clip(np.searchsorted(times, query_t, side="right") - 1, 0, len(times) - 1)
    if lag_ms <= 1e-5:
        return command[indices]
    tau = lag_ms * .001
    decay = np.exp(-np.diff(times) / tau)
    band = np.ones((2, len(times)))
    band[1, :-1] = -decay
    rhs = np.empty(len(times))
    rhs[0] = command[0]
    rhs[1:] = -np.expm1(-np.diff(times) / tau) * command[:-1]
    state = solve_banded((1, 0), band, rhs, check_finite=False)
    remaining = np.maximum(query_t - times[indices], 0.)
    alpha = np.exp(-remaining / tau)
    return state[indices] * alpha + command[indices] * (1 - alpha)


def identify(segments, bootstrap=12):
    train = [part for part in segments if not part["validation"]]
    holdout = [part for part in segments if part["validation"]]
    if ({part["stage"] for part in train} != {0, 1, 2}
            or {part["stage"] for part in holdout} != {3, 4, 5}):
        return {"accepted": False, "reason": "Need three complete training and three independent validation segments"}
    y = np.concatenate([part["measured"] for part in train])
    if np.std(y) < 1e-3 or any(np.std(part["command"]) < 1e-3 for part in train):
        return {"accepted": False, "reason": "Insufficient command or reported-current excitation"}

    def prediction(parts, parameters):
        return np.concatenate([response(part["command_t"], part["command"], part["feedback_t"], *parameters) for part in parts])

    def project(parameters, weights=None):
        model = prediction(train, parameters)
        design = np.column_stack((model, np.ones(len(model))))
        selected = np.arange(len(y)) if weights is None else weights
        coefficients = np.linalg.lstsq(design[selected], y[selected], rcond=None)[0]
        return (design @ coefficients - y)[selected], coefficients

    candidates = []
    for delay in np.arange(0., 60.1, 4.):
        for lag in (.001, .5, 2., 5., 12., 30., 60.):
            residual, _ = project((delay, lag))
            candidates.append((float(residual @ residual), delay, lag))
    best = None
    for _, delay, lag in sorted(candidates)[:3]:
        fit = least_squares(lambda p: project(p)[0], [delay, lag], bounds=([0., 1e-6], [60., 80.]),
                            x_scale=[5., 5.], diff_step=1e-3, max_nfev=100)
        if best is None or fit.cost < best.cost:
            best = fit
    residual, (gain, bias) = project(best.x)
    validation = np.concatenate([part["measured"] for part in holdout])
    predicted_validation = gain * prediction(holdout, best.x) + bias
    validation_metrics = metrics(validation, predicted_validation)
    per_segment = [dict(stage=part["stage"], **metrics(part["measured"], gain * prediction([part], best.x) + bias)) for part in holdout]
    singular = np.linalg.svd(best.jac, compute_uv=False)
    conditioning = float(singular[0] / max(singular[-1], 1e-15))
    intervals = None
    if bootstrap:
        blocks, offset = [], 0
        for part in train:
            bins = ((part["feedback_t"] - part["feedback_t"][0]) / .1).astype(int)
            blocks += [np.flatnonzero(bins == block) + offset for block in np.unique(bins)]
            offset += len(bins)
        rng = np.random.default_rng(20261110)
        draws = []
        for _ in range(bootstrap):
            selected = np.concatenate([blocks[index] for index in rng.integers(0, len(blocks), len(blocks))])
            fit = least_squares(lambda p: project(p, selected)[0], best.x, bounds=([0., 1e-6], [60., 80.]),
                                x_scale=[5., 5.], diff_step=1e-3, max_nfev=40)
            draws.append(fit.x.tolist())
        intervals = {"delay_ms": np.percentile(draws, [2.5, 97.5], axis=0)[:, 0].tolist(),
                     "lag_ms": np.percentile(draws, [2.5, 97.5], axis=0)[:, 1].tolist(),
                     "draws": bootstrap, "method": "100 ms case blocks in training only; conditional on timestamps and one-pole model"}
    delay, lag = map(float, best.x)
    accepted = (gain > 0 and validation_metrics["nrmse"] <= .2
                and all(part["nrmse"] <= .3 for part in per_segment)
                and delay < 59.8 and lag < 79.8 and conditioning < 1000.)
    return {"accepted": bool(accepted), "scope": "host frame assembly to reported current-derived torque; transport and telemetry included",
            "delay_ms": delay, "lag_ms": lag, "gain": float(gain), "bias_nm": float(bias),
            "train": metrics(y, y + residual), "holdout": validation_metrics, "holdout_segments": per_segment,
            "delay_lag_jacobian_condition": conditioning, "conditional_95_percent_interval": intervals,
            "reason": None if accepted else "Held-out model error, sign, parameter boundary or delay/lag ambiguity failed acceptance",
            "pure_foc_delay_ms": None, "physical_one_way_communication_ms": None,
            "uncertainty_note": "Subsample fitting does not establish subsample hardware timestamp accuracy or separate FOC from telemetry"}


def frequency_response(segments):
    spectra = []
    rates = []
    for part in segments:
        dt = max(np.median(np.diff(part["feedback_t"])), np.median(np.diff(part["command_t"])))
        grid = np.arange(part["feedback_t"][0], part["feedback_t"][-1], dt)
        if len(grid) < 256:
            continue
        u = part["command"][np.clip(np.searchsorted(part["command_t"], grid, side="right") - 1, 0, len(part["command"]) - 1)]
        y = np.interp(grid, part["feedback_t"], part["measured"])
        nperseg = min(1024, len(grid) // 3)
        f, pxx = welch(u, fs=1 / dt, nperseg=nperseg)
        _, pyy = welch(y, fs=1 / dt, nperseg=nperseg)
        _, pxy = csd(u, y, fs=1 / dt, nperseg=nperseg)
        coherence = abs(pxy) ** 2 / np.maximum(pxx * pyy, 1e-24)
        valid = (f > 0) & (f <= .1 / dt) & (pxx > np.max(pxx) * .01)
        transfer = pxy / np.maximum(pxx, 1e-24)
        spectra.append({"stage": part["stage"], "frequency_hz": f[valid].tolist(),
                        "coherence": coherence[valid].tolist(), "gain": abs(transfer[valid]).tolist(),
                        "phase_rad": np.angle(transfer[valid]).tolist()})
        rates.append(float(1 / dt))
    return {"resampling": "ZOH input and interpolated reported current; band limited to min observed rate / 10",
            "minimum_observed_rate_hz": min(rates) if rates else None, "segments": spectra}


def axis_result(events, axis, expected_segment_s=10., bootstrap=12):
    selected = events[events["axis"] == axis]
    all_tx = np.sort(selected[selected["event"] == 1], order="steady_ns")
    tx = np.sort(selected[(selected["event"] == 1) & (selected["code"] == 1)], order="steady_ns")
    rx = np.sort(selected[selected["event"] == 2], order="steady_ns")
    markers = np.sort(selected[selected["event"] == 4], order="steady_ns")
    if len(tx) < 300 or len(rx) < 300 or not len(markers):
        return {"accepted": False, "reason": "No complete latency experiment for this axis", "command_events": len(tx), "feedback_events": len(rx)}
    errors = []
    for name, records in (("command", all_tx), ("feedback", rx)):
        finite_records = records[records["code"] == 1] if name == "command" else records
        if not np.all(np.isfinite(finite_records["value"])):
            errors.append(name + " contains nonfinite samples")
        if np.any(np.diff(records["steady_ns"].astype(np.int64)) <= 0):
            errors.append(name + " timestamps are not strictly increasing")
        if np.any(np.diff(records["sequence"].astype(np.int64)) != 1):
            errors.append(name + " sequence has missing, duplicate or reordered frames")
    if axis < 4 and np.any(rx["code"] >= 8):
        errors.append("Drive fault present in recording")
    if np.any(markers["end_steady_ns"] < markers["steady_ns"]) or np.any(markers["code"] % 100 > 5):
        errors.append("Invalid control timestamp order or stage")
    origin = min(int(tx["steady_ns"][0]), int(rx["steady_ns"][0]))
    ct = (tx["steady_ns"].astype(np.int64) - origin) * 1e-9
    ft = (rx["steady_ns"].astype(np.int64) - origin) * 1e-9
    marker_t = (markers["steady_ns"].astype(np.int64) - origin) * 1e-9
    boundaries = np.r_[0, np.flatnonzero((np.diff(marker_t) > .05)
                    | (np.diff(markers["code"] % 100) != 0) | (np.diff(markers["sequence"].astype(np.int64)) != 0)) + 1, len(markers)]
    segments, rejected = [], []
    for begin, end in zip(boundaries[:-1], boundaries[1:]):
        stage = int(markers["code"][begin] % 100)
        start, stop = marker_t[begin], marker_t[end - 1]
        if stop - start < expected_segment_s - .05 or np.any(markers["code"][begin:end] >= 100):
            rejected.append({"stage": stage, "reason": "Incomplete segment or command saturation"})
            continue
        command = (ct >= start - .5) & (ct <= stop)
        feedback = (ft >= start + .1) & (ft <= stop)
        if np.count_nonzero(feedback) < 300 or np.count_nonzero(command) < 300:
            rejected.append({"stage": stage, "reason": "Too few independent events"})
            continue
        segments.append({"stage": stage, "validation": stage >= 3,
                         "command_t": ct[command], "command": tx["value"][command],
                         "feedback_t": ft[feedback], "measured": rx["value"][feedback]})
    fit = identify(segments, bootstrap) if not errors else {"accepted": False, "reason": "; ".join(errors)}
    fit.update(command_events=len(tx), feedback_events=len(rx), complete_segments=len(segments), rejected_segments=rejected,
               command_interval_ms=summary(np.diff(ct) * 1000), feedback_interval_ms=summary(np.diff(ft) * 1000),
               host_control_compute_ms=summary(milliseconds(markers["end_steady_ns"], markers["steady_ns"])))
    ends = np.sort(markers["end_steady_ns"])
    indices = np.searchsorted(ends, tx["steady_ns"], side="right") - 1
    in_range = indices >= 0
    queue_delay = milliseconds(tx["steady_ns"][in_range], ends[indices[in_range]])
    fit["controller_to_frame_assembly_ms"] = summary(queue_delay[(queue_delay >= 0) & (queue_delay < 2.)])
    if segments:
        fit["frequency_response"] = frequency_response(segments)
    return fit


def deployment_result(path, events):
    base = "/wheel_leg/rl/"
    actions, first_pd, accepts, frames, pd_events = {}, {}, {}, {}, {}
    counters, previous_counters = {}, {}
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            if None in row or any(value is None for value in row.values()):
                raise ValueError("Incomplete deployment CSV")
            def number(suffix):
                value = row.get(base + suffix, "0")
                return int(value) if value not in ("nan", "") else 0
            frame = number("input/frame_steady_ns")
            if frame:
                frames[frame] = number("input/frame_source_oldest_ns")
            sequence = number("action_seq")
            if sequence and float(row.get(base + "valid", "0")) > 0:
                actions.setdefault(sequence, [number("latency/" + suffix) for suffix in (
                    "source_frame_ns", "queued_ns", "inference_start_ns", "inference_end_ns", "result_ready_ns", "first_publish_ns")])
            accepted = number("latency/accepted_action_seq")
            if accepted:
                accepts.setdefault(accepted, number("latency/action_accept_ns"))
            applied = number("latency/first_pd_action_seq")
            if applied:
                first_pd.setdefault(applied, number("latency/first_pd_ns"))
            pd_seq = number("latency/pd_sequence")
            if pd_seq and number("latency/pd_start_ns"):
                pd_events[(pd_seq, number("latency/pd_start_ns"))] = milliseconds(number("latency/pd_end_ns"), number("latency/pd_start_ns"))
            for key in ("submit_attempts", "submit_rejected", "pending_overwritten", "result_overwritten", "replaced_before_pd"):
                current = number("latency/" + key)
                previous = previous_counters.get(key, 0)
                counters[key] = counters.get(key, 0) + (current - previous if current >= previous else current)
                previous_counters[key] = current
    steps = {key: [] for key in ("source_age_at_frame", "frame_to_worker_queue", "worker_queue", "inference", "result_pack",
                                   "result_to_first_publish", "publish_to_accept", "accept_to_first_pd", "frame_to_first_pd")}
    axis_queue = {name: [] for name in AXES}
    axis_total = {name: [] for name in AXES}
    origin, first_frames = None, {}
    for event in events:
        if event["event"] == 5:
            origin = (int(event["sequence"]), int(event["end_steady_ns"])) if event["code"] == 1 else None
        elif event["event"] == 3:
            origin = None
        elif event["event"] == 1 and event["code"] == 1 and origin:
            sequence, pd_end = origin
            axis = int(event["axis"])
            if (sequence in first_pd and pd_end >= first_pd[sequence]
                    and int(event["steady_ns"]) >= pd_end):
                first_frames.setdefault((sequence, axis), int(event["steady_ns"]))
    for sequence, stamps in actions.items():
        if len(stamps) != 6 or not all(stamps) or any(b < a for a, b in zip(stamps, stamps[1:])):
            continue
        for key, start, end in zip(list(steps)[1:6], stamps[:-1], stamps[1:]):
            steps[key].append(float(milliseconds(end, start)))
        if frames.get(stamps[0], 0) and frames[stamps[0]] <= stamps[0]:
            steps["source_age_at_frame"].append(float(milliseconds(stamps[0], frames[stamps[0]])))
        if sequence in accepts and accepts[sequence] >= stamps[-1]:
            steps["publish_to_accept"].append(float(milliseconds(accepts[sequence], stamps[-1])))
        if sequence not in first_pd or sequence not in accepts or first_pd[sequence] < accepts[sequence]:
            continue
        steps["accept_to_first_pd"].append(float(milliseconds(first_pd[sequence], accepts[sequence])))
        steps["frame_to_first_pd"].append(float(milliseconds(first_pd[sequence], stamps[0])))
        for axis, name in enumerate(AXES):
            if (sequence, axis) in first_frames:
                frame_ns = first_frames[sequence, axis]
                axis_queue[name].append(float(milliseconds(frame_ns, first_pd[sequence])))
                axis_total[name].append(float(milliseconds(frame_ns, stamps[0])))
    return {"observed_actions": len(actions), "observed_first_pd_actions": len(first_pd), "counters": counters,
            "latency_ms": {name: summary(values) for name, values in steps.items()},
            "pd_compute_ms": summary(list(pd_events.values())),
            "first_pd_to_first_host_frame_assembly_ms": {name: summary(values) for name, values in axis_queue.items()},
            "frame_to_first_host_frame_assembly_ms": {name: summary(values) for name, values in axis_total.items()},
            "scope": "software timestamps in this process; event 5 carries action provenance into host frame assembly; no CAN or drive acknowledgement"}


def analyze(path, control=None, expected_segment_s=10., bootstrap=12):
    path = Path(path)
    events = read_events(path)
    footer = events[events["event"] == 9]
    valid_trace = len(footer) == 1 and int(footer["sequence"][0]) == 0 and int(events["event"][-1]) == 9
    result = {"schema": "wheel_leg_latency_result_v1", "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "expected_segment_duration_s": expected_segment_s,
              "clean_trace": bool(valid_trace), "trace_dropped": int(footer["sequence"][0]) if len(footer) == 1 else None,
              "host_command_batch_ms": None, "axes": {}, "deployment": None,
              "unidentified": {"usb_mcu_can_downlink_ms": None, "can_to_actual_current_foc_ms": None,
                               "telemetry_sample_filter_uplink_ms": None, "physical_closed_loop_delay_ms": None},
              "runtime_admissible": False}
    batches = events[events["event"] == 3]
    result["host_command_batch_ms"] = summary(milliseconds(batches["end_steady_ns"], batches["steady_ns"]))
    for axis, name in enumerate(AXES):
        fit = axis_result(events, axis, expected_segment_s, bootstrap)
        if not valid_trace:
            fit["accepted"] = False
            fit["trace_error"] = "Missing clean-close footer or dropped events; timing fit is diagnostic only"
        result["axes"][name] = fit
    if control:
        result["deployment"] = deployment_result(control, events)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("events", type=Path)
    parser.add_argument("--control", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bootstrap", type=int, default=12)
    parser.add_argument("--expected-segment-duration", type=float)
    args = parser.parse_args()
    parameters_path = args.events.parent / "parameters.yaml"
    duration = args.expected_segment_duration if args.expected_segment_duration is not None else 10.
    if parameters_path.exists():
        import yaml
        parameters = yaml.safe_load(parameters_path.read_text())
        frozen_duration = parameters.get("latency_probe", {}).get("ros__parameters", {}).get("segment_duration_s", duration)
        if args.expected_segment_duration is not None and frozen_duration != duration:
            raise ValueError("Requested segment duration differs from frozen capture parameters")
        duration = frozen_duration
    result = analyze(args.events, args.control, duration, args.bootstrap)
    manifest_path = args.events.parent / "manifest.json"
    if manifest_path.exists():
        result["capture_manifest"] = json.loads(manifest_path.read_text())
        result["capture_parameters_match"] = (parameters_path.exists() and
            result["capture_manifest"]["parameters_sha256"] == hashlib.sha256(parameters_path.read_bytes()).hexdigest())
        if not result["capture_parameters_match"]:
            for fit in result["axes"].values():
                fit["accepted"] = False
                fit["contract_error"] = "Capture parameters differ from frozen manifest"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    for axis, fit in result["axes"].items():
        if "delay_ms" in fit:
            print(f"{axis}: effective delay={fit['delay_ms']:.3f} ms, lag={fit['lag_ms']:.3f} ms, accepted={fit['accepted']}")
    print(args.output)


if __name__ == "__main__":
    main()
