#!/usr/bin/env python3
"""Offline latency recovery, trace integrity, holdout and deployment pairing tests."""

import csv
import json
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

from analyze_actuator_latency import DTYPE, analyze, axis_result, deployment_result, identify, read_events, response
from prepare_actuator_latency import prepare


def truth_response(command_t, command, sample_t, delay, lag):
    shifted = command_t + delay
    state, index, previous = command[0], 0, shifted[0]
    output = []
    for current in sample_t:
        while index + 1 < len(command) and shifted[index + 1] <= current:
            end = shifted[index + 1]
            state = command[index] + (state - command[index]) * np.exp(-(end - previous) / lag)
            previous = end
            index += 1
        value = command[index] + (state - command[index]) * np.exp(-max(current - previous, 0.) / lag)
        output.append(value)
    return np.array(output)


def synthetic_events():
    rng = np.random.default_rng(6741)
    origin = 576460752303423495
    delays_ms = [1.6, 2.2, 2.8, 3.4, 3.5, 5.5]
    lags_ms = [1.8, 2.4, 3.2, 4., 2., 3.]
    records = []
    for axis in range(6):
        ct = np.arange(0, 20., .001)
        ct += rng.uniform(-.00006, .00006, len(ct))
        ct[0] = 0.
        bits = rng.choice([-1., 1.], 2100)
        command = bits[np.minimum((ct / .01).astype(int), len(bits) - 1)] * (1. if axis < 4 else .3)
        ft = np.arange(.0012, 19.999, .001) + rng.uniform(-.00012, .00012, 19998)
        measured = .94 * truth_response(ct, command, ft, delays_ms[axis] / 1000., lags_ms[axis] / 1000.) + .08
        measured += rng.normal(0., .002, len(ft))
        for seq, (t, value) in enumerate(zip(ct, command), 1):
            records.append((1, axis, seq, origin + int(t * 1e9), 0, value, 1))
        for seq, (t, value) in enumerate(zip(ft, measured), 1):
            records.append((2, axis, seq, origin + int(t * 1e9), 0, value, 1 if axis < 4 else 0))
        for stage in range(6):
            for t in np.arange(2.5 + stage * 3., 4.5 + stage * 3., .001):
                records.append((4, axis, 1, origin + int(t * 1e9), origin + int(t * 1e9) + 15000, 1., stage))
    records.sort(key=lambda row: row[3])
    records.append((9, -1, 0, origin + 21_000_000_000, 0, 0., 0))
    return np.array(records, dtype=DTYPE), delays_ms, lags_ms


def save_events(path, events):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([name for name, _ in DTYPE])
        for event in events:
            writer.writerow([int(event[name]) if name != "value" else float(event[name]) for name, _ in DTYPE])


class LatencyAnalysisTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.events, cls.delays, cls.lags = synthetic_events()
        cls.directory = tempfile.TemporaryDirectory()
        cls.path = Path(cls.directory.name) / "synthetic_events.csv"
        save_events(cls.path, cls.events)
        cls.result = analyze(cls.path, expected_segment_s=2., bootstrap=0)
        report = os.environ.get("RMCS_LATENCY_TEST_REPORT")
        if report:
            directory = Path(report)
            directory.mkdir(parents=True, exist_ok=True)
            (directory / "synthetic_events.csv").write_bytes(cls.path.read_bytes())
            (directory / "synthetic_result.json").write_text(json.dumps(dict(data_kind="synthetic", truth_delay_ms=cls.delays,
                truth_lag_ms=cls.lags, result=cls.result), indent=2) + "\n")

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_recovers_six_independent_delays_with_irregular_sampling(self):
        for axis, fit in enumerate(self.result["axes"].values()):
            with self.subTest(axis=axis):
                self.assertTrue(fit["accepted"], fit)
                self.assertAlmostEqual(fit["delay_ms"], self.delays[axis], delta=.15)
                self.assertAlmostEqual(fit["lag_ms"], self.lags[axis], delta=.2)
                self.assertEqual(fit["complete_segments"], 6)
                self.assertLess(fit["holdout"]["nrmse"], .04)
                self.assertIsNone(fit["pure_foc_delay_ms"])
        self.assertFalse(self.result["runtime_admissible"])

    def test_preserves_integer_timestamps_above_double_precision(self):
        read = read_events(self.path)
        self.assertEqual(int(read["steady_ns"][0]), 576460752303423495)
        np.testing.assert_array_equal(read["steady_ns"], self.events["steady_ns"])

    def test_exact_nonuniform_filter_matches_independent_event_integrator(self):
        ct = np.array([0., .001, .0031, .005, .0093])
        command = np.array([1., -1., .4, .9, -.2])
        ft = np.arange(.001, .02, .00037)
        expected = truth_response(ct, command, ft, .0023, .0017)
        np.testing.assert_allclose(response(ct, command, ft, 2.3, 1.7), expected, atol=1e-12)

    def test_missing_frame_is_not_silently_interpolated(self):
        events = self.events.copy()
        index = np.flatnonzero((events["axis"] == 0) & (events["event"] == 2))[400]
        result = axis_result(np.delete(events, index), 0, 2., 0)
        self.assertFalse(result["accepted"])
        self.assertIn("sequence", result["reason"])

    def test_saturated_or_partial_segments_do_not_count_as_holdout(self):
        events = self.events.copy()
        mask = (events["axis"] == 0) & (events["event"] == 4) & (events["code"] == 4)
        events["code"][mask] = 104
        result = axis_result(events, 0, 2., 0)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["complete_segments"], 5)

    def test_constant_or_absent_validation_is_unidentifiable(self):
        self.assertFalse(identify([], 0)["accepted"])
        segments = [dict(stage=i, validation=i >= 3, command_t=np.arange(1000) * .001,
                         command=np.ones(1000), feedback_t=np.arange(1000) * .001, measured=np.ones(1000)) for i in range(6)]
        self.assertFalse(identify(segments, 0)["accepted"])

    def test_non_torque_enable_frame_is_not_counted_as_a_lost_torque_frame(self):
        events = self.events.copy()
        index = np.flatnonzero((events["axis"] == 0) & (events["event"] == 1))[0]
        events["code"][index] = 2
        events["value"][index] = np.nan
        self.assertTrue(axis_result(events, 0, 2., 0)["accepted"])

    def test_rejects_model_that_does_not_predict_independent_validation(self):
        events = self.events.copy()
        mask = (events["axis"] == 0) & (events["event"] == 2) & (events["steady_ns"] >= 576460763803423495)
        events["value"][mask] *= -1
        result = axis_result(events, 0, 2., 0)
        self.assertFalse(result["accepted"])
        self.assertGreater(result["holdout"]["nrmse"], 1.)

    def test_reports_conditional_interval_without_calling_it_foc(self):
        result = axis_result(self.events, 0, 2., 6)
        self.assertTrue(result["accepted"])
        self.assertEqual(result["conditional_95_percent_interval"]["draws"], 6)
        self.assertIsNone(result["pure_foc_delay_ms"])
        self.assertIn("timestamp", result["uncertainty_note"])

    def test_truncated_or_overflow_trace_cannot_be_accepted(self):
        for variant in (self.events[:-1], self.events.copy()):
            if variant["event"][-1] == 9:
                variant["sequence"][-1] = 1
            path = Path(self.directory.name) / "incomplete_events.csv"
            save_events(path, variant)
            result = analyze(path, expected_segment_s=2., bootstrap=0)
            self.assertFalse(result["clean_trace"])
            self.assertTrue(all(not part["accepted"] for part in result["axes"].values()))

    def test_prepare_separates_six_axes_and_freezes_provenance(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "capture"
            manifest, _ = prepare(root, capture, "right_knee_joint")
            self.assertEqual(manifest["axis"], "right_knee_joint")
            self.assertTrue((capture / "parameters.yaml").exists())
            self.assertIn("can_to_current_foc_only", manifest["unknown"])
            with self.assertRaises(ValueError):
                prepare(root, capture)

    def test_first_use_is_counted_once_when_action_is_held(self):
        base = "/wheel_leg/rl/"
        columns = {"valid": "1", "action_seq": "7", "input/frame_steady_ns": "1000000000",
                   "input/frame_source_oldest_ns": "999000000", "latency/source_frame_ns": "1000000000",
                   "latency/queued_ns": "1000100000", "latency/inference_start_ns": "1000200000",
                   "latency/inference_end_ns": "1000400000", "latency/result_ready_ns": "1000450000",
                   "latency/first_publish_ns": "1001000000", "latency/accepted_action_seq": "7",
                   "latency/action_accept_ns": "1001100000", "latency/first_pd_action_seq": "7", "latency/first_pd_ns": "1005100000"}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "deployment.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=[base + key for key in columns])
                writer.writeheader()
                for _ in range(30):
                    writer.writerow({base + key: value for key, value in columns.items()})
            events = np.array([
                (1, 0, 1, 1005150000, 0, .1, 1),
                (5, -1, 7, 1005000000, 1005100000, np.nan, 1),
                (1, 0, 2, 1005200000, 0, .1, 1),
                (1, 1, 1, 1011100000, 0, .1, 1),
                (3, -1, 1, 1005150000, 1011200000, np.nan, 0)], dtype=DTYPE)
            result = deployment_result(path, events)
        self.assertEqual(result["latency_ms"]["accept_to_first_pd"]["count"], 1)
        self.assertAlmostEqual(result["latency_ms"]["accept_to_first_pd"]["p50"], 4.)
        self.assertAlmostEqual(result["latency_ms"]["frame_to_first_pd"]["p50"], 5.1)
        self.assertAlmostEqual(result["first_pd_to_first_host_frame_assembly_ms"]["left_hip_joint"]["p50"], .1)
        self.assertAlmostEqual(result["first_pd_to_first_host_frame_assembly_ms"]["left_knee_joint"]["p50"], 6.)


if __name__ == "__main__":
    unittest.main()
