#!/usr/bin/env python3
"""Synthetic time/quantization/held-out contracts; these are not bench results."""

import csv
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / ".script/identification"
sys.path.insert(0, str(SCRIPTS))
import fit_current_sweep as fit
from current_actuator_model import CurrentActuatorModel
from prepare_current_sweep import generate


def synthesize(path: Path):
    prefixes = [f"/chassis/{j}_joint" for j in fit.JOINTS]
    columns = ["schema_version", "host_time_s", "stage", "point_index", "active_joint",
               "actuation_active", "amplitude_raw", "frequency_hz", "queue_dropped"]
    suffixes = ("physical_angle", "physical_velocity", "current_raw", "temperature",
                "feedback_sequence", "feedback_time_s", "command_sequence", "command_prepare_time_s",
                "command_current_raw", "command_limited", "current_clipped", "current_rate_limited",
                "sweep_probe_current_raw")
    columns.extend(f"{p}/{s}" for p in prefixes for s in suffixes)
    dt, tau, delay_samples = 0.004, 0.018, 2
    gain, bias = 1.1, -3.0
    coefficient = np.array([-0.02, 0.003, 0.001, -0.8, -0.2, -1.5, 0.4, 0.2])
    q, v, iq, t = 0.55, 0.0, 0.0, 100.0
    sequence = 0
    history = [0.0] * 3
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, columns)
        writer.writeheader()
        for point, target in enumerate((0.55, 0.85, 1.15)):
            for stage, duration in ((2, 1.0), (3, 6.0), (2, 1.0), (4, 3.0)):
                for k in range(int(duration / dt)):
                    elapsed = k * dt
                    f = 0.2 + (8.0 - 0.2) * elapsed / 6.0
                    probe = (-192 * math.sin(2 * math.pi * (0.2 * elapsed + 0.5 * 7.8 / 6 * elapsed ** 2))
                             if stage == 3 else (-192 if int(elapsed / 0.25) % 2 else 192) if stage == 4 else 0)
                    u = float(np.round(np.clip(-5000 * (target - q) + 600 * v + probe, -2048, 2048)))
                    # Feedback at t+dt reflects the command prepared at t-delay.
                    applied = history[-delay_samples]
                    previous_iq = iq
                    a = math.exp(-dt / tau)
                    iq = a * iq + (1 - a) * (gain * applied + bias)
                    history.append(u)
                    history = history[-4:]
                    def acceleration(angle, velocity):
                        current = (iq + previous_iq) / 2
                        x = [current, current * (angle - 0.85), current * abs(velocity), velocity,
                             math.tanh(10 * velocity), math.sin(angle), math.cos(angle), 1]
                        return float(np.dot(x, coefficient))
                    acc = acceleration(q, v)
                    mid_q, mid_v = q + 0.5 * dt * v, v + 0.5 * dt * acc
                    q += dt * mid_v
                    v += dt * acceleration(mid_q, mid_v)
                    t += dt
                    sequence += 1
                    for duplicate in range(2):
                        row = {name: 0.0 for name in columns}
                        row.update(schema_version=1, host_time_s=t + duplicate * dt / 2 + 0.0001,
                                   stage=stage, point_index=point, active_joint=0, actuation_active=1,
                                   amplitude_raw=192, frequency_hz=f if stage == 3 else 0,
                                   queue_dropped=0)
                        for j, p in enumerate(prefixes):
                            row.update({p + "/" + name: value for name, value in {
                                "physical_angle": q if j == 0 else target,
                                "physical_velocity": v if j == 0 else 0,
                                "current_raw": iq if j == 0 else 0,
                                "temperature": 30, "feedback_sequence": sequence,
                                "feedback_time_s": t, "command_sequence": sequence,
                                "command_prepare_time_s": t - dt,
                                "command_current_raw": u if j == 0 else 0,
                                "sweep_probe_current_raw": probe if j == 0 else 0,
                                # Deliberately reject one short saturated interval.
                                "current_clipped": 1 if point == 1 and 1 < elapsed < 1.04 else 0,
                            }.items()})
                        writer.writerow(row)
    metadata = {"schema_version": 1, "command_domain": "raw_current", "condition": "suspended",
                "max_command_current_raw": 2048, "max_feedback_age_s": 0.05,
                "current_to_physical_direction": [-1] * 4, "targets": prefixes}
    Path(str(path) + ".json").write_text(json.dumps(metadata))
    Path(str(path) + ".status.json").write_text(json.dumps({
        "completed": True, "write_failed": False, "dropped_samples": 0}))
    return coefficient, tau, delay_samples * dt


class CurrentSweepFitTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.csv = Path(cls.temp.name) / "synthetic.csv"
        cls.expected, cls.tau, cls.delay = synthesize(cls.csv)
        cls.output = Path(cls.temp.name) / "model.json"
        cls.result = fit.fit_file(cls.csv, cls.output, ["left_front"])

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_fit_uses_distinct_feedback_and_actual_time_and_held_out_rollout(self):
        self.assertEqual(self.result["failures"], {})
        model = self.result["models"]["left_front"]
        self.assertTrue(model["training_ready"], model)
        self.assertFalse(model["absolute_torque_identified"])
        self.assertGreater(model["quality"]["duplicate_feedback"], 100)
        self.assertGreater(model["quality"]["rejected_samples"], 0)
        self.assertGreater(model["validation"]["r2"], 0.9)
        self.assertGreater(model["transport"]["validation"]["r2"], 0.9)
        self.assertLess(abs(model["transport"]["delay_s"] - self.delay), 0.006)
        self.assertLess(abs(model["transport"]["time_constant_s"] - self.tau), 0.012)
        # Kinematics identifies gain/inertia, not Kt or J separately.
        self.assertAlmostEqual(model["coefficients"][0], self.expected[0], delta=0.003)
        self.assertLess(model["validation_rollout"]["angle"]["rmse"], 0.01)
        pack = np.load(self.output.with_suffix(".npz"), allow_pickle=False)
        self.assertTrue(np.all(pack["left_front_train"][:, 6] == 3))
        self.assertTrue(np.all(pack["left_front_validation"][:, 6] == 4))

    def test_consumer_enforces_condition_coverage_and_explicit_simulation_inertia(self):
        model = CurrentActuatorModel(self.output, "left_front", "suspended")
        current = model.step_current(128.5, 0.004)
        self.assertTrue(np.isfinite(current))
        self.assertEqual(model.history[-1][1], 129)
        q, v, iq = 0.85, 0.0, -20.0
        acceleration = model.acceleration(q, v, iq)
        self.assertAlmostEqual(model.equivalent_simulation_torque(q, v, iq, 0.2, -0.1),
                               0.2 * acceleration + 0.1)
        with self.assertRaises(ValueError):
            model.acceleration(2.0, v, iq)
        with self.assertRaises(ValueError):
            CurrentActuatorModel(self.output, "left_front", "grounded")

    def test_partial_logs_and_queue_loss_cannot_be_training_ready(self):
        sidecar = Path(str(self.csv) + ".status.json")
        original = sidecar.read_text()
        try:
            sidecar.write_text(json.dumps({"completed": False, "write_failed": False, "dropped_samples": 0}))
            with self.assertRaises(ValueError):
                fit.load_dataset(self.csv)
            data = fit.load_dataset(self.csv, allow_partial=True)
            model, _ = fit.fit_joint(data, 0)
            self.assertFalse(model["training_ready"])
            sidecar.write_text(json.dumps({"completed": True, "write_failed": False, "dropped_samples": 1}))
            with self.assertRaises(ValueError):
                fit.load_dataset(self.csv, allow_partial=True)
        finally:
            sidecar.write_text(original)

    def test_unexcited_data_fails_identifiability(self):
        data = fit.load_dataset(self.csv)
        data.samples[0][:, 1] = 0.8
        data.samples[0][:, 2] = 0.0
        with self.assertRaisesRegex(ValueError, "excitation"):
            fit.fit_joint(data, 0)

    def test_legacy_nm_fitter_rejects_count_csv(self):
        spec = importlib.util.spec_from_file_location("legacy_fit", SCRIPTS / "fit_sweep_graybox.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        with self.assertRaisesRegex(ValueError, "current counts"):
            module.read_rows(self.csv)

    def test_configs_copy_actual_hardware_and_adrc_and_have_no_nm_clamps(self):
        config_dir = ROOT / "rmcs_ws/src/rmcs_bringup/config"
        plugin_types = {p.attrib["type"] for p in
                        ET.parse(ROOT / "rmcs_ws/src/rmcs_core/plugins.xml").getroot()}
        controllers = ("lf_joint_controller", "lb_joint_controller", "rb_joint_controller", "rf_joint_controller")
        for normal in config_dir.glob("deformable-infantry-omni*.yaml"):
            document = yaml.safe_load(normal.read_text())
            for controller in controllers:
                p = document[controller]["ros__parameters"]
                self.assertEqual(p["output_domain"], "current_raw")
                self.assertTrue(p["control"].endswith("/control_current_raw"))
                self.assertTrue(p["applied_current_raw"].endswith("/command_current_raw"))
                self.assertFalse(set(p) & {"u_min", "u_max", "output_min", "output_max"})
        source = yaml.safe_load((config_dir / "deformable-infantry-omni-rl.yaml").read_text())
        for condition in ("suspended", "grounded"):
            config = generate(source, condition, [65, 50, 35, 25, 35, 50, 65], [128, 384])
            committed = yaml.safe_load((ROOT / f"rmcs_ws/src/rmcs_bringup/config/deformable-current-sweep-{condition}.yaml").read_text())
            self.assertEqual(config, committed)
            components = config["rmcs_executor"]["ros__parameters"]["components"]
            for c in components:
                self.assertIn(c.split("->")[0].strip(), plugin_types)
            self.assertIn("DeformableInfantryOmniC", components[0])
            self.assertFalse(any("RlBridge" in c or "PidController" in c for c in components))
            p = config["current_sweep"]["ros__parameters"]
            self.assertEqual(p["max_command_current_raw"], 2048)
            self.assertEqual(p["max_current_slew_raw_s"], 0)
            snapshot = json.loads(p["experiment_config_json"])
            for controller in ("lf_joint_controller", "lb_joint_controller", "rb_joint_controller", "rf_joint_controller"):
                p = config[controller]["ros__parameters"]
                self.assertEqual(snapshot["adrc"][controller], p)
                self.assertEqual(p["b0"], source[controller]["ros__parameters"]["b0"])
                self.assertTrue(p["control"].endswith("/servo_current_raw"))
                self.assertFalse(set(p) & {"u_min", "u_max", "output_min", "output_max"})


if __name__ == "__main__":
    unittest.main()
