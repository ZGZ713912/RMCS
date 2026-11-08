#!/usr/bin/env python3
"""Analyze revision-8 installed-leg breakaway, crawl and holding-load experiments."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from analyze_joint_sweep import S, read_csv


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def unique_frames(indices, sequence):
    return indices[np.r_[True, np.diff(sequence[indices]) > 0]] if len(indices) else indices


def analyze(data, incomplete=(), tolerance_nm=.5):
    report = {"schema": "wheel_leg_installed_characterization_v1", "accepted": False,
              "runtime_admissible": False, "real2sim_ready": False,
              "incomplete_records": list(incomplete), "rejected_stages": [],
              "breakaway_trials": [], "breakaway_pairs": [], "crawl_trials": [],
              "moving_friction_pairs": [], "friction_models": [],
              "holding_trials": [], "holding_map": [], "slow_motion_loads": [], "slow_motion_comparisons": [],
              "holdout_tolerance_nm": tolerance_nm,
              "unidentified": {"physical_spring_force": None, "seal_friction": None,
                  "output_backlash_rad": None, "transmission_stiffness": None,
                  "independent_shaft_torque_scale": None},
              "scope": "installed effective current-derived motor-coordinate loads; no automatic model application",
              "compensation_contract": "use measured TOTAL torque; never subtract controller load I from plant input"}
    required = [S + name for name in ("state", "reference_active", "side", "fixture", "run_id", "mode",
                "stage", "waveform", "excitation_revision", "validation", "control_steady_ns",
                "stage_elapsed_s", "stage_duration_s", "probe_axis", "approach_direction",
                "breakaway_phase", "breakaway_origin_rad", "breakaway_event_elapsed_s", "beta_reference")]
    missing = [key for key in required if key not in data]
    if missing:
        report["reason"] = "missing revision-8 metadata: " + ", ".join(missing)
        return report
    active = (data[S + "state"] == 3) & (data[S + "reference_active"] == 1)
    if not np.any(active):
        report["reason"] = "no active experiment"
        return report
    if len(np.unique(data[S + "run_id"][active])) != 1:
        report["reason"] = "analyze one active run at a time; select --run-id to avoid mixing starts"
        return report
    sides, fixtures = np.unique(data[S + "side"][active]), np.unique(data[S + "fixture"][active])
    if len(sides) != 1 or sides[0] not in (0, 1) or len(fixtures) != 1 or fixtures[0] not in (0, 1):
        report["reason"] = "one side and one declared fixture are required per recording"
        return report
    side = "left" if sides[0] == 0 else "right"
    report["side"], report["fixture"] = side, "horizontal" if fixtures[0] == 0 else "vertical"
    axes = [f"{side}_{joint}_joint" for joint in ("hip", "knee")]
    for axis in axes:
        required += [S + axis + "/" + field for field in ("model_angle", "model_velocity", "model_sign",
                    "reference", "reference_velocity", "reference_acceleration", "tracking_error",
                    "saturated", "load_compensation_model")]
        required += [f"/wheel_leg/{axis}/" + field for field in ("velocity", "torque", "feedback_sequence",
                    "feedback_steady_ns", "tx_kind", "tx_queued_steady_ns", "fault_code", "temperature_mos")]
    required.append(S + side + "/beta_estimate")
    missing = [key for key in required if key not in data]
    if missing:
        report["reason"] = "missing signals: " + ", ".join(missing)
        return report
    controls = data[S + "control_steady_ns"]
    if np.any(np.diff(controls) <= 0):
        report["reason"] = "nonmonotonic control timestamps; split recordings before analysis"
        return report
    q = np.column_stack([data[S + a + "/model_angle"] for a in axes])
    v = np.column_stack([data[S + a + "/model_velocity"] for a in axes])
    signs = np.column_stack([data[S + a + "/model_sign"] for a in axes])
    api_v = np.column_stack([data[f"/wheel_leg/{a}/velocity"] for a in axes])
    tau = signs * np.column_stack([data[f"/wheel_leg/{a}/torque"] for a in axes])
    if not np.all(np.abs(signs[active]) == 1) or not np.allclose(v[active], signs[active] * api_v[active], equal_nan=False):
        report["reason"] = "model/API velocity signs are inconsistent"
        return report
    stamps = np.column_stack([data[f"/wheel_leg/{a}/feedback_steady_ns"] for a in axes])
    seq = np.column_stack([data[f"/wheel_leg/{a}/feedback_sequence"] for a in axes])
    sat = np.column_stack([data[S + a + "/saturated"] for a in axes]).any(axis=1)
    faults = np.column_stack([data[f"/wheel_leg/{a}/fault_code"] for a in axes]).any(axis=1)
    errors = np.column_stack([data[S + a + "/tracking_error"] for a in axes])
    temperature = np.column_stack([data[f"/wheel_leg/{a}/temperature_mos"] for a in axes])
    tx = np.column_stack([data[f"/wheel_leg/{a}/tx_queued_steady_ns"] for a in axes])
    tx_kind = np.column_stack([data[f"/wheel_leg/{a}/tx_kind"] for a in axes])
    valid = (np.isfinite(q).all(axis=1) & np.isfinite(v).all(axis=1) & np.isfinite(tau).all(axis=1)
             & np.isfinite(temperature).all(axis=1) & np.isfinite(errors).all(axis=1) & (seq > 0).all(axis=1)
             & ((controls[:, None] - stamps >= 0) & (controls[:, None] - stamps < 5_000_000)).all(axis=1)
             & (np.abs(stamps[:, 0] - stamps[:, 1]) < 2_000_000) & ~sat & ~faults
             & (tx_kind == 1).all(axis=1) & (tx > 0).all(axis=1)
             & (tx <= controls[:, None] + 5_000_000).all(axis=1)
             & (tx > controls[:, None] - 5_000_000).all(axis=1))
    beta = np.rad2deg(data[S + side + "/beta_estimate"])
    valid &= (np.isfinite(beta) & (beta >= 39.5) & (beta <= 110.5)
              & np.isfinite(data[S + "beta_reference"]))
    report["feedback"] = {}
    for j, axis in enumerate(axes):
        differences = np.diff(seq[active, j])
        report["feedback"][axis] = {"unique_frames": int(len(np.unique(seq[active, j]))),
            "repeated_fraction": float(np.mean(differences == 0)) if len(differences) else 0.0}
    for run, stage in np.unique(np.column_stack((data[S + "run_id"][active], data[S + "stage"][active])), axis=0):
        mask = active & (data[S + "run_id"] == run) & (data[S + "stage"] == stage)
        ii = np.flatnonzero(mask)
        waveforms = np.unique(data[S + "waveform"][ii])
        if len(waveforms) != 1 or waveforms[0] not in (8, 9, 10, 11):
            continue
        waveform = int(waveforms[0])
        row = {"run": int(run), "stage": int(stage), "waveform": waveform,
               "validation": bool(data[S + "validation"][ii[0]]),
               "direction": int(data[S + "approach_direction"][ii[0]])}
        elapsed, duration = data[S + "stage_elapsed_s"][ii], data[S + "stage_duration_s"][ii]
        reason = None
        if not np.all(data[S + "excitation_revision"][ii] == 8):
            reason = "requires revision 8"
        elif (not np.all(np.isfinite(elapsed)) or not np.all(np.isfinite(duration)) or duration[0] <= 0
              or np.any(np.diff(elapsed) < 0)
              or len(np.unique(data[S + "validation"][ii])) != 1 or len(np.unique(duration)) != 1
              or len(np.unique(data[S + "approach_direction"][ii])) != 1):
            reason = "stage metadata changes within segment"
        elif elapsed.min() > .025 or elapsed.max() < duration[0] - .025 or np.any(np.diff(controls[ii]) > 5_000_000):
            reason = "partial segment or control gap over 5 ms"
        elif np.any(sat[ii]) or np.any(faults[ii]) or np.mean(valid[ii]) < .95:
            reason = "saturation, fault or insufficient fresh paired/encoded data"
        if reason:
            report["rejected_stages"].append(dict(row, reason=reason))
            continue
        row["beta_target_deg"] = float(np.rad2deg(np.median(data[S + "beta_reference"][ii])))
        if waveform in (8, 9):
            axis = int(data[S + "probe_axis"][ii[0]])
            if axis not in (0, 1) or not np.all(data[S + "probe_axis"][ii] == axis) or row["direction"] not in (-1, 1):
                report["rejected_stages"].append(dict(row, reason="invalid probe axis/direction"))
                continue
            row["axis"] = axes[axis]
            fresh = unique_frames(ii[valid[ii]], seq[:, axis])
            if waveform == 8:
                phase = data[S + "breakaway_phase"]
                ramp = fresh[phase[fresh] == 1]
                terminal = fresh[np.isin(phase[fresh], (2, 3, 4, 5))]
                load_i = data[S + row["axis"] + "/load_compensation_model"][ii]
                if not len(terminal) or len(ramp) < 30 or not np.all(np.isfinite(load_i)) or np.ptp(load_i) > 1e-8:
                    report["rejected_stages"].append(dict(row, reason="invalid/short trial or unfrozen integral"))
                    continue
                end = terminal[0]
                phase_end = int(phase[end])
                if (np.any(phase[ii[ii >= end]] != phase_end)
                    or data[S + "breakaway_event_elapsed_s"][end] < 0):
                    report["rejected_stages"].append(dict(row, reason="inconsistent breakaway phase/event history"))
                    continue
                stationary = ramp[(np.abs(v[ramp, axis]) <= .02)
                    & (np.abs(q[ramp, axis] - data[S + "breakaway_origin_rad"][ramp]) < .003)]
                if len(stationary) < 10:
                    report["rejected_stages"].append(dict(row, reason="no stationary threshold bracket"))
                    continue
                last = stationary[-5:]
                row.update(outcome={2: "motion_detected", 3: "right_censored", 4: "unsettled_entry", 5: "wrong_direction"}[phase_end],
                    endpoint_torque_model_nm=float(np.median(tau[last, axis])),
                    first_motion_torque_model_nm=float(tau[end, axis]) if phase_end == 2 else None,
                    event_elapsed_s=float(data[S + "breakaway_event_elapsed_s"][end]),
                    q_rad=np.median(q[ramp[:10]], axis=0).tolist(),
                    temperature_c=float(np.median(temperature[ramp, axis])),
                    beta_actual_deg=float(np.median(beta[ramp[:10]])))
                report["breakaway_trials"].append(row)
            else:
                plateau = fresh[(data[S + "stage_elapsed_s"][fresh] > .25)
                    & (data[S + "stage_elapsed_s"][fresh] < duration[0] - .25)]
                if len(plateau) < 50:
                    report["rejected_stages"].append(dict(row, reason="too few independent crawl plateau frames"))
                    continue
                target = data[S + row["axis"] + "/reference_velocity"][plateau]
                speed = float(np.median(np.abs(target)))
                frame_time = (stamps[fresh, axis] - stamps[fresh[0], axis]) * 1e-9
                lo = np.searchsorted(frame_time, frame_time - .1)
                hi = np.minimum(np.searchsorted(frame_time, frame_time + .1), len(fresh) - 1)
                span = frame_time[hi] - frame_time[lo]
                encoder_speed = np.divide(q[fresh[hi], axis] - q[fresh[lo], axis], span,
                                          out=np.full(len(fresh), np.nan), where=span >= .1)
                measured = encoder_speed[np.searchsorted(fresh, plateau)]
                moving = row["direction"] * measured
                if not np.all(np.isfinite(moving)):
                    report["rejected_stages"].append(dict(row, reason="insufficient encoder displacement window"))
                    continue
                center = np.median(data[S + row["axis"] + "/reference"][ii])
                cells = []
                for position in np.arange(-.035, .036, .005):
                    indices = plateau[(np.abs(q[plateau, axis] - center - position) < .0025)
                                      & (moving > .2 * speed)]
                    if len(indices) >= 3:
                        cells.append({"bin": round(float(position), 4), "torque_nm": float(np.median(tau[indices, axis])),
                                      "q_rad": np.median(q[indices], axis=0).tolist(),
                                      "speed_rad_s": float(np.median(np.abs(measured[np.searchsorted(plateau, indices)])))})
                row.update(speed_target_rad_s=speed, speed_actual_rad_s=float(np.median(moving)),
                    stuck_fraction=float(np.mean(moving < .2 * speed)),
                    speed_error_rms_rad_s=float(np.sqrt(np.mean((measured - target) ** 2))),
                    drive_velocity_vs_encoder_rms_rad_s=float(np.sqrt(np.mean((v[plateau, axis] - measured) ** 2))),
                    velocity_source="encoder displacement over 0.1--0.2 s windows; drive velocity retained in raw CSV",
                    temperature_c=float(np.median(temperature[plateau, axis])), cells=cells)
                report["crawl_trials"].append(row)
        else:
            paired = []
            for i in ii[valid[ii]]:
                if not paired or np.all(seq[i] > seq[paired[-1]]):
                    paired.append(i)
            fresh = np.asarray(paired, dtype=int)
            if waveform == 10:
                tail = fresh[data[S + "stage_elapsed_s"][fresh] >= duration[0] - 3]
                if len(tail) < 100:
                    report["rejected_stages"].append(dict(row, reason="too few independent settled hold frames"))
                    continue
                time = (controls[tail] - controls[tail[0]]) * 1e-9
                slope = np.linalg.lstsq(np.column_stack((time, np.ones(len(time)))), tau[tail], rcond=None)[0][0]
                if (np.max(np.abs(v[tail])) > .02 or np.max(np.abs(errors[tail])) > .01
                    or np.max(np.abs(slope)) > .05 or np.ptp(beta[tail]) > .5
                    or abs(np.median(beta[tail]) - row["beta_target_deg"]) > .5):
                    report["rejected_stages"].append(dict(row, reason="holding tail has not settled"))
                    continue
                row.update(torque_model_nm=np.median(tau[tail], axis=0).tolist(),
                    q_rad=np.median(q[tail], axis=0).tolist(), beta_actual_deg=float(np.median(beta[tail])),
                    torque_slope_nm_s=slope.tolist(), temperature_c=np.median(temperature[tail], axis=0).tolist())
                report["holding_trials"].append(row)
            else:
                if not len(fresh) or row["direction"] * (beta[fresh[-1]] - beta[fresh[0]]) < 65:
                    report["rejected_stages"].append(dict(row, reason="slow loop did not traverse the requested beta range/direction"))
                    continue
                for center in range(45, 110, 5):
                    indices = fresh[(np.abs(beta[fresh] - center) < 2.5)]
                    if len(indices) < 50:
                        continue
                    report["slow_motion_loads"].append(dict(row, half_period_s=float(duration[0]), beta_bin_deg=center,
                        beta_actual_deg=float(np.median(beta[indices])),
                        q_rad=np.median(q[indices], axis=0).tolist(),
                        speed_abs_rad_s=np.median(np.abs(v[indices]), axis=0).tolist(),
                        temperature_c=np.median(temperature[indices], axis=0).tolist(),
                        torque_model_nm=np.median(tau[indices], axis=0).tolist(),
                        scope="spring, moving friction and residual inertia remain combined"))
    pair_friction(report)
    build_holding_map(report)
    moving_models(report)
    compare_slow_motion(report)
    breakaway_cells = {(p["axis"], p["beta_deg"], p["validation"]) for p in report["breakaway_pairs"] if p["accepted"]}
    crawl_cells = {(p["axis"], p["beta_deg"], p["validation"], p["speed_target_rad_s"])
                   for p in report["moving_friction_pairs"] if p["accepted"]}
    report["coverage"] = {
        "breakaway": {"accepted_pairs": len(breakaway_cells), "required_pairs": 12},
        "crawl": {"accepted_pairs": len(crawl_cells), "required_pairs": 60},
        "holding": {"accepted_knots": sum(p["accepted"] for p in report["holding_map"]), "required_knots": 8},
        "slow_motion": {"accepted_pairs": sum(p["accepted"] for p in report["slow_motion_comparisons"]), "required_pairs": 52}}
    modes = set(int(x) for x in data.get(S + "mode", [0])[active]) if S + "mode" in data else set()
    relevant = ([("breakaway", 12), ("crawl", 60)] if 4 in modes else [])
    relevant += [("holding", 8), ("slow_motion", 52)] if 5 in modes else []
    report["accepted"] = bool(relevant and all(report["coverage"][key].get("accepted_pairs", report["coverage"][key].get("accepted_knots", 0)) >= count
                                              for key, count in relevant)
                               )
    report["remaining"] = [key for key, value in report["coverage"].items()
        if value.get("accepted_pairs", value.get("accepted_knots", value.get("observed_stages", 0)))
        < value.get("required_pairs", value.get("required_knots", value.get("required_stages", 0)))]
    return report


def pair_friction(report):
    for key, output in (("breakaway_trials", "breakaway_pairs"), ("crawl_trials", "moving_friction_pairs")):
        groups = {}
        for row in report[key]:
            center = min((50, 75, 100), key=lambda value: abs(row["beta_target_deg"] - value))
            speed = round(row.get("speed_target_rad_s", 0), 4)
            group = (row["run"], row["axis"], center, row["validation"], speed)
            groups.setdefault(group, {})[row["direction"]] = row
        for (run, axis, beta, validation, speed), rows in groups.items():
            pair = {"run": run, "axis": axis, "beta_deg": beta, "validation": validation, "accepted": False}
            if set(rows) != {-1, 1}:
                report[output].append(dict(pair, reason="missing opposite direction"))
                continue
            positive, negative = rows[1], rows[-1]
            if abs(positive["temperature_c"] - negative["temperature_c"]) > 3:
                report[output].append(dict(pair, reason="direction-pair temperature difference exceeds 3 C"))
                continue
            if key == "breakaway_trials":
                if any(row["outcome"] != "motion_detected" for row in rows.values()):
                    report[output].append(dict(pair, reason="censored or invalid breakaway; no finite static threshold"))
                    continue
                if np.max(np.abs(np.array(positive["q_rad"]) - negative["q_rad"])) > .005:
                    report[output].append(dict(pair, reason="breakaway directions sampled different actual poses"))
                    continue
                high, low = positive["endpoint_torque_model_nm"], negative["endpoint_torque_model_nm"]
                pair.update(positive_endpoint_nm=high, negative_endpoint_nm=low,
                    half_width_nm=(high - low) / 2, center_nm=(high + low) / 2,
                    scope="operational stationary torque interval, including installed load/history and drive resolution")
            else:
                pair["speed_target_rad_s"] = speed
                if max(positive["stuck_fraction"], negative["stuck_fraction"]) > .1:
                    report[output].append(dict(pair, reason="stick-slip dominates; retain diagnostic trajectory, do not fit viscous friction"))
                    continue
                if any(abs(row["speed_actual_rad_s"] - speed) > max(.003, .3 * speed) for row in rows.values()):
                    report[output].append(dict(pair, reason="requested crawl-speed band was not achieved"))
                    continue
                pos, neg = ({cell["bin"]: cell for cell in row["cells"]} for row in (positive, negative))
                matched = [(pos[b], neg[b]) for b in pos.keys() & neg.keys()
                    if np.max(np.abs(np.array(pos[b]["q_rad"]) - neg[b]["q_rad"])) <= .005
                    and abs(pos[b]["speed_rad_s"] - neg[b]["speed_rad_s"]) <= max(.003, .3 * speed)]
                if len(matched) < 3:
                    report[output].append(dict(pair, reason="too few matched actual-pose/speed direction cells"))
                    continue
                pair.update(half_width_nm=float(np.median([(p["torque_nm"] - n["torque_nm"]) / 2 for p, n in matched])),
                    speed_actual_rad_s=float(np.median([(p["speed_rad_s"] + n["speed_rad_s"]) / 2 for p, n in matched])),
                    matched_cells=len(matched))
                pair["temperature_c"] = (positive["temperature_c"] + negative["temperature_c"]) / 2
            pair["accepted"] = pair["half_width_nm"] > 0
            report[output].append(pair)
    for pair in report["breakaway_pairs"]:
        if pair["validation"] and pair["accepted"]:
            train = [p for p in report["breakaway_pairs"] if not p["validation"] and p["accepted"]
                     and (p["run"], p["axis"], p["beta_deg"]) == (pair["run"], pair["axis"], pair["beta_deg"])]
            pair["accepted"] = bool(len(train) == 1 and abs(train[0]["half_width_nm"] - pair["half_width_nm"]) <= report["holdout_tolerance_nm"])


def moving_models(report):
    for axis, beta in sorted({(p["axis"], p["beta_deg"]) for p in report["moving_friction_pairs"]}):
        rows = [p for p in report["moving_friction_pairs"] if p["axis"] == axis and p["beta_deg"] == beta and p["accepted"]]
        train, holdout = ([p for p in rows if p["validation"] == flag] for flag in (False, True))
        model = {"axis": axis, "beta_deg": beta, "accepted": False, "coulomb_nm": None,
                 "viscous_nm_s_rad": None, "stribeck_velocity_rad_s": None,
                 "scope": "optional effective Coulomb-plus-viscous model; low-speed curve retained even on rejection"}
        if len(train) >= 5 and len(holdout) >= 5:
            from scipy.optimize import lsq_linear
            speed = np.array([p["speed_actual_rad_s"] for p in train])
            x = np.column_stack((np.ones(len(speed)), speed))
            y = np.array([p["half_width_nm"] for p in train])
            coefficients = lsq_linear(x, y, bounds=(0, np.inf)).x
            error = np.array([p["half_width_nm"] - coefficients @ [1, p["speed_actual_rad_s"]] for p in holdout])
            model["holdout_rmse_nm"] = float(np.sqrt(np.mean(error ** 2)))
            model["train_rmse_nm"] = float(np.sqrt(np.mean((x @ coefficients - y) ** 2)))
            model["holdout_max_error_nm"] = float(np.max(np.abs(error)))
            model["train_max_error_nm"] = float(np.max(np.abs(x @ coefficients - y)))
            model["observed_speed_range_rad_s"] = [float(speed.min()), float(speed.max())]
            model["accepted"] = bool(min(np.linalg.svd(x / np.linalg.norm(x, axis=0), compute_uv=False)) > .05
                and max(model["holdout_max_error_nm"], model["train_max_error_nm"]) <= report["holdout_tolerance_nm"]
                and abs(np.median([p["temperature_c"] for p in train]) - np.median([p["temperature_c"] for p in holdout])) <= 3)
            if model["accepted"]:
                model.update(coulomb_nm=float(coefficients[0]), viscous_nm_s_rad=float(coefficients[1]))
        report["friction_models"].append(model)


def build_holding_map(report):
    groups = {}
    for row in report["holding_trials"]:
        beta = round(row["beta_target_deg"])
        groups.setdefault(beta, []).append(row)
    for beta, rows in sorted(groups.items()):
        train = [r for r in rows if not r["validation"]]
        testing = [r for r in rows if r["validation"]]
        knot = {"beta_target_deg": beta, "accepted": False, "directions": [],
                "scope": "motor-coordinate installed holding map; gravity/CAD correction required for vertical fixture"}
        for row in testing:
            matches = [r for r in train if r["direction"] == row["direction"]]
            if len(matches) != 1:
                continue
            fitted = matches[0]
            pose_error = float(np.max(np.abs(np.array(row["q_rad"]) - fitted["q_rad"])))
            error = np.array(row["torque_model_nm"]) - fitted["torque_model_nm"]
            temp_delta = float(np.max(np.abs(np.array(row["temperature_c"]) - fitted["temperature_c"])))
            knot["directions"].append({"direction": row["direction"], "q_rad": fitted["q_rad"],
                "beta_actual_deg": fitted["beta_actual_deg"], "torque_model_nm": fitted["torque_model_nm"],
                "holdout_error_nm": error.tolist(), "pose_error_rad": pose_error,
                "temperature_difference_c": temp_delta,
                "accepted": bool(pose_error <= .01 and temp_delta <= 3 and np.max(np.abs(error)) <= report["holdout_tolerance_nm"])})
        labels = {d["direction"] for d in knot["directions"] if d["accepted"]}
        expected = {0, -1} if beta == 40 else {0, 1} if beta == 110 else {-1, 1}
        knot["accepted"] = expected.issubset(labels)
        if {-1, 1}.issubset(labels):
            up, down = (next(d for d in knot["directions"] if d["direction"] == sign) for sign in (1, -1))
            knot["direction_mean_nm"] = ((np.array(up["torque_model_nm"]) + down["torque_model_nm"]) / 2).tolist()
            knot["direction_half_difference_nm"] = ((np.array(up["torque_model_nm"]) - down["torque_model_nm"]) / 2).tolist()
        report["holding_map"].append(knot)


def compare_slow_motion(report):
    groups = {}
    for row in report["slow_motion_loads"]:
        groups.setdefault((row["direction"], row["half_period_s"], row["beta_bin_deg"]), []).append(row)
    for (direction, half_period, beta), rows in sorted(groups.items()):
        train, testing = ([r for r in rows if r["validation"] == flag] for flag in (False, True))
        pair = {"direction": direction, "half_period_s": half_period, "beta_bin_deg": beta, "accepted": False}
        if len(train) == len(testing) == 1:
            error = np.array(testing[0]["torque_model_nm"]) - train[0]["torque_model_nm"]
            temperature_delta = np.array(testing[0]["temperature_c"]) - train[0]["temperature_c"]
            speed_delta = np.array(testing[0]["speed_abs_rad_s"]) - train[0]["speed_abs_rad_s"]
            pair.update(holdout_error_nm=error.tolist(), accepted=bool(np.max(np.abs(error)) <= report["holdout_tolerance_nm"]
                and np.max(np.abs(temperature_delta)) <= 3 and np.max(np.abs(speed_delta)) <= .01
                and np.max(np.abs(np.array(testing[0]["q_rad"]) - train[0]["q_rad"])) <= .01
                and abs(testing[0]["beta_actual_deg"] - train[0]["beta_actual_deg"]) <= .5))
        report["slow_motion_comparisons"].append(pair)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, action="append", default=[])
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--run-id", type=int)
    parser.add_argument("--holdout-torque-tolerance-nm", type=float, default=.5)
    args = parser.parse_args()
    if not np.isfinite(args.holdout_torque_tolerance_nm) or args.holdout_torque_tolerance_nm <= 0:
        parser.error("holdout tolerance must be finite and positive")
    data, incomplete = read_csv(args.csv)
    if args.run_id is not None and S + "run_id" in data:
        selected = data[S + "run_id"] == args.run_id
        if not np.any(selected):
            parser.error("requested run-id is absent from recording")
        data = {key: value[selected] for key, value in data.items()}
    report = analyze(data, incomplete, args.holdout_torque_tolerance_nm)
    report["collection_contract_verified"] = False
    if args.manifest:
        manifest = json.loads(args.manifest.read_text())
        contract = manifest["contract_id"]
        recording_contract = data.get(S + "contract_id", np.array([0]))
        parameters_path = args.manifest.parent / manifest["parameters"]
        matches = bool(contract > 0 and np.all(recording_contract == contract)
                       and digest(parameters_path) == manifest["parameters_sha256"]
                       and manifest["side"] == report.get("side") and manifest["fixture"] == report.get("fixture"))
        report["collection_contract_verified"] = matches
        report["collection_manifest"] = {"path": str(args.manifest.resolve()), "sha256": digest(args.manifest),
                                         "source_head": manifest["source_head"], "source_dirty": manifest["source_dirty"]}
        if not matches:
            report["accepted"] = False
            report["contract_rejection"] = "CSV contract, side/fixture or frozen parameter hash mismatch"
    report["source"] = {"path": str(args.csv.resolve()), "sha256": digest(args.csv)}
    report["analysis_source_sha256"] = {name: digest(Path(__file__).parent / name)
        for name in ("analyze_leg_characterization.py", "analyze_joint_sweep.py")}
    report["configuration_files"] = [{"path": str(p.resolve()), "sha256": digest(p)} for p in args.config]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps({key: report[key] for key in ("accepted", "real2sim_ready", "runtime_admissible")}
                     | {"output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
