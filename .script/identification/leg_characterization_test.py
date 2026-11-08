#!/usr/bin/env python3
"""Synthetic protocol/analysis checks; never represent these fixtures as vehicle data."""

import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from analyze_joint_sweep import S, read_csv
from analyze_leg_characterization import analyze
from prepare_leg_characterization import prepare


class Recording:
    def __init__(self, side="left"):
        self.side = side
        self.parts = []
        self.count = 0

    def stage(self, waveform, duration, validation=False, axis=0, direction=1, beta=75,
              speed=.05, censored=False):
        t = np.arange(0, duration - 1e-6, .005)
        n = len(t)
        stamp = 18_014_398_509_481_985 + (self.count + np.arange(n, dtype=np.int64)) * 5_000_000
        q = np.tile([0., np.deg2rad(beta)], (n, 1))
        v = np.zeros((n, 2))
        reference_v = v.copy()
        tau = np.tile([4., -8.], (n, 1))
        phase = np.zeros(n)
        event = np.full(n, -1.)
        beta_values = np.full(n, float(beta))
        if waveform == 8:
            event_time = 6 if censored else 2
            phase[:] = np.where(t < event_time, 1, 3 if censored else 2)
            event[t >= event_time] = event_time
            tau[:, axis] += direction * np.minimum(t, event_time)
            if not censored:
                q[t >= event_time, axis] += direction * .0031
                v[t >= event_time, axis] = direction * .05
        elif waveform == 9:
            ramp = .2
            displacement = np.where(t < ramp, speed * (t / 2 - ramp / (2 * np.pi) * np.sin(np.pi * t / ramp)),
                speed * (t - ramp / 2))
            tail = duration - t
            mask = t > duration - ramp
            displacement[mask] = .12 - speed * (tail[mask] / 2 - ramp / (2 * np.pi) * np.sin(np.pi * tail[mask] / ramp))
            q[:, axis] += direction * (displacement - .06)
            reference_v[:, axis] = direction * speed
            tau[:, axis] += direction * (2 + .4 * speed)
        elif waveform == 10:
            tau = np.tile([1 + beta * .02 + direction * .2, -5 + beta * .03 - direction * .3], (n, 1))
        elif waveform == 11:
            blend = (t / duration) ** 3 * (10 - 15 * t / duration + 6 * (t / duration) ** 2)
            beta_values = 40 + 70 * blend if direction > 0 else 110 - 70 * blend
            q[:, 1] = np.deg2rad(beta_values)
            tau = np.column_stack((1 + beta_values * .02 + direction * .3,
                                   -5 + beta_values * .03 - direction * .4))
        data = {S + "state": np.full(n, 3), S + "reference_active": np.ones(n),
                S + "side": np.full(n, self.side == "right"), S + "fixture": np.zeros(n),
                S + "run_id": np.ones(n), S + "stage": np.full(n, len(self.parts)),
                S + "waveform": np.full(n, waveform), S + "excitation_revision": np.full(n, 8),
                S + "validation": np.full(n, validation), S + "control_steady_ns": stamp,
                S + "contract_id": np.full(n, 576_460_752_303_423_495, dtype=np.int64),
                S + "stage_elapsed_s": t, S + "stage_duration_s": np.full(n, duration),
                S + "probe_axis": np.full(n, axis), S + "approach_direction": np.full(n, direction),
                S + "breakaway_phase": phase, S + "breakaway_origin_rad": np.full(n, 0 if axis == 0 else np.deg2rad(beta)),
                S + "breakaway_event_elapsed_s": event, S + "beta_reference": np.deg2rad(beta_values),
                S + self.side + "/beta_estimate": np.deg2rad(beta_values), S + "mode": np.full(n, 4 if waveform < 10 else 5)}
        for j, joint in enumerate(("hip", "knee")):
            motor = f"/wheel_leg/{self.side}_{joint}_joint/"
            sweep = S + f"{self.side}_{joint}_joint/"
            data.update({sweep + "model_angle": q[:, j], sweep + "model_velocity": v[:, j],
                sweep + "model_sign": np.full(n, -1), sweep + "reference": q[:, j],
                sweep + "reference_velocity": reference_v[:, j], sweep + "reference_acceleration": np.zeros(n),
                sweep + "tracking_error": np.zeros(n), sweep + "saturated": np.zeros(n),
                sweep + "load_compensation_model": np.full(n, 7.),
                motor + "velocity": -v[:, j], motor + "torque": -tau[:, j],
                motor + "feedback_sequence": self.count + np.arange(n) + 1,
                motor + "feedback_steady_ns": stamp, motor + "tx_kind": np.ones(n),
                motor + "tx_queued_steady_ns": stamp + 200_000, motor + "fault_code": np.zeros(n),
                motor + "temperature_mos": np.full(n, 25.)})
        self.parts.append(data)
        self.count += n

    def data(self):
        return {key: np.concatenate([part[key] for part in self.parts]) for key in self.parts[0]}


class LegCharacterizationTest(unittest.TestCase):
    def test_breakaway_is_total_torque_interval_and_censoring_is_not_a_point_fit(self):
        rec = Recording("right")
        for validation in (False, True):
            for direction in (1, -1):
                rec.stage(8, 8, validation, direction=direction)
        data = rec.data()
        result = analyze(data)
        self.assertEqual(len(result["breakaway_pairs"]), 2, result)
        for pair in result["breakaway_pairs"]:
            self.assertTrue(pair["accepted"], pair)
            self.assertAlmostEqual(pair["half_width_nm"], 2, delta=.025)
            self.assertAlmostEqual(pair["center_nm"], 4)
        data[S + "right_hip_joint/load_compensation_model"].fill(15)
        invariant = analyze(data)
        self.assertEqual(result["breakaway_pairs"], invariant["breakaway_pairs"])
        censored = Recording()
        for direction in (1, -1):
            censored.stage(8, 8, direction=direction, censored=True)
        result = analyze(censored.data())
        self.assertTrue(all(t["outcome"] == "right_censored" for t in result["breakaway_trials"]))
        self.assertFalse(any(p["accepted"] for p in result["breakaway_pairs"]))
        self.assertFalse(result["real2sim_ready"])

    def test_crawl_uses_encoder_motion_and_fits_only_with_independent_whole_strokes(self):
        rec = Recording()
        for validation in (False, True):
            for speed in (.01, .025, .05, .1, .2):
                for direction in (1, -1):
                    rec.stage(9, .12 / speed + .2, validation, direction=direction, speed=speed)
        result = analyze(rec.data())
        self.assertEqual(len(result["moving_friction_pairs"]), 10, result)
        self.assertTrue(all(p["accepted"] for p in result["moving_friction_pairs"]), result)
        model = result["friction_models"][0]
        self.assertTrue(model["accepted"], model)
        self.assertAlmostEqual(model["coulomb_nm"], 2, places=2)
        self.assertAlmostEqual(model["viscous_nm_s_rad"], .4, places=2)
        self.assertIsNone(model["stribeck_velocity_rad_s"])

    def test_complete_friction_project_requires_all_poses_axes_and_repetitions(self):
        rec = Recording()
        for beta in (50, 75, 100):
            for validation in (False, True):
                for axis in (0, 1):
                    for direction in (1, -1):
                        rec.stage(8, 8, validation, axis=axis, direction=direction, beta=beta)
                    for speed in (.01, .025, .05, .1, .2):
                        for direction in (1, -1):
                            rec.stage(9, .12 / speed + .2, validation, axis=axis,
                                      direction=direction, beta=beta, speed=speed)
        data = rec.data()
        result = analyze(data)
        self.assertTrue(result["accepted"], result["coverage"])
        self.assertEqual(result["coverage"]["breakaway"]["accepted_pairs"], 12)
        self.assertEqual(result["coverage"]["crawl"]["accepted_pairs"], 60)
        self.assertEqual(len(result["friction_models"]), 6)
        selected = (data[S + "waveform"] == 8) & (data[S + "validation"] == 1) & (data[S + "probe_axis"] == 1)
        data[S + "left_knee_joint/saturated"][np.flatnonzero(selected)[0]] = 1
        result = analyze(data)
        self.assertFalse(result["accepted"])
        self.assertIn("breakaway", result["remaining"])

    def test_holding_map_and_slow_loops_require_both_approaches_and_holdout(self):
        rec = Recording()
        current = 40
        for validation in (False, True):
            for direction in (1, -1):
                for beta in (range(40, 111, 10) if direction > 0 else range(110, 39, -10)):
                    approach = direction if current != beta else 0
                    rec.stage(10, 6, validation, direction=approach, beta=beta)
                    current = beta
        for validation in (False, True):
            for duration in (60, 15):
                for direction in (1, -1):
                    rec.stage(11, duration, validation, direction=direction)
        result = analyze(rec.data())
        self.assertEqual(result["coverage"]["holding"]["accepted_knots"], 8, result)
        self.assertEqual(result["coverage"]["slow_motion"]["accepted_pairs"], 52, result)
        self.assertTrue(result["accepted"], result)
        self.assertFalse(result["runtime_admissible"])
        self.assertFalse(result["real2sim_ready"])
        data = rec.data()
        drift = (data[S + "validation"] == 1) & (data[S + "waveform"] == 10)
        data["/wheel_leg/left_knee_joint/torque"][drift] += 2
        self.assertFalse(analyze(data)["accepted"])

    def test_partial_saturated_and_stale_segments_do_not_fit(self):
        for failure in ("partial", "single_record", "saturation", "stale", "integral"):
            rec = Recording()
            rec.stage(8, 8)
            data = rec.data()
            if failure == "partial":
                data = {key: value[100:] for key, value in data.items()}
            elif failure == "single_record":
                data = {key: value[:1] for key, value in data.items()}
            elif failure == "saturation":
                data[S + "left_hip_joint/saturated"][20] = 1
            elif failure == "stale":
                data["/wheel_leg/left_hip_joint/feedback_steady_ns"] -= 10_000_000
            else:
                data[S + "left_hip_joint/load_compensation_model"] += np.linspace(0, 1, len(data[S + "state"]))
            result = analyze(data)
            self.assertEqual(result["breakaway_trials"], [], (failure, result))
            self.assertTrue(result["rejected_stages"])
            json.dumps(result, allow_nan=False)

    def test_csv_reader_retains_new_metadata_and_integer_receipts(self):
        rec = Recording()
        rec.stage(8, 8)
        data = rec.data()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "synthetic.csv"
            with path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(data.keys())
                for index in range(len(data[S + "state"])):
                    writer.writerow([int(value[index]) if value.dtype.kind in "biu" else value[index] for value in data.values()])
            restored, incomplete = read_csv(path)
        self.assertEqual(int(restored[S + "control_steady_ns"][0]), 18_014_398_509_481_985)
        self.assertEqual(int(restored[S + "contract_id"][0]), 576_460_752_303_423_495)
        self.assertFalse(incomplete)
        self.assertEqual(len(analyze(restored)["breakaway_trials"]), 1)

    def test_frozen_collection_bundle_and_csv_contract_must_match(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as folder:
            bundle = Path(folder) / "experiment"
            manifest, command = prepare(root, "friction", "left", "horizontal", bundle)
            self.assertFalse(manifest["hardware_started"])
            self.assertIn("--params-file", command)
            rec = Recording()
            rec.stage(8, 8)
            data = rec.data()
            data[S + "contract_id"].fill(manifest["contract_id"])
            with (bundle / "raw.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(data.keys())
                for index in range(len(data[S + "state"])):
                    writer.writerow([int(value[index]) if value.dtype.kind in "biu" else value[index] for value in data.values()])
            args = [sys.executable, str(root / ".script/identification/analyze_leg_characterization.py"),
                    str(bundle / "raw.csv"), "--manifest", str(bundle / "manifest.json"),
                    "--output", str(bundle / "analysis.json")]
            subprocess.run(args, check=True, capture_output=True)
            self.assertTrue(json.loads((bundle / "analysis.json").read_text())["collection_contract_verified"])
            with (bundle / "parameters.yaml").open("a") as stream:
                stream.write("\n# modified after capture\n")
            subprocess.run(args, check=True, capture_output=True)
            changed = json.loads((bundle / "analysis.json").read_text())
            self.assertFalse(changed["collection_contract_verified"])
            self.assertFalse(changed["accepted"])


if __name__ == "__main__":
    unittest.main()
