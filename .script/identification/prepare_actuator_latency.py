#!/usr/bin/env python3
"""Freeze a latency capture or deployment trace profile without starting the robot."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import shlex
import subprocess

import yaml

AXES = ("left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint", "left_wheel", "right_wheel")


def prepare(root, output, axis="left_wheel", amplitude=None, wheel_velocity=0., deployment=None,
            bit_duration=.01, segment_duration=10., excursion=.1):
    root, output = Path(root).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError("Capture directory already exists; choose a new directory")
    config_path = Path(deployment).resolve() if deployment else root / "rmcs_ws/src/rmcs_bringup/config/wheel-leg-latency.yaml"
    config = yaml.safe_load(config_path.read_text())
    config["wheel_leg"]["ros__parameters"]["latency_trace_path"] = str(output / "events.csv")
    collector = config.setdefault("value_collector", {}).setdefault("ros__parameters", {})
    collector.update(csv_path=str(output / "control.csv"), write_interval=1, flush_interval=1000, strict_signals=True)
    signals = collector.setdefault("signals", [])
    components = config["rmcs_executor"]["ros__parameters"]["components"]
    if not any("ValueCollector" in component for component in components):
        components.append("rmcs_core::debug::ValueCollector -> value_collector")
    if deployment:
        if not any("WheelLegRlController" in component for component in components):
            raise ValueError("Deployment tracing needs an existing WheelLegRlController configuration")
        signals += ["/wheel_leg/rl/" + name for name in (
            "valid", "action_seq", "action_steady_ns", "input/frame_sequence", "input/frame_steady_ns",
            "input/frame_source_oldest_ns", "latency/queued_ns", "latency/inference_start_ns",
            "latency/inference_end_ns", "latency/result_ready_ns", "latency/first_publish_ns",
            "latency/source_frame_ns", "latency/submit_attempts", "latency/submit_rejected",
            "latency/pending_overwritten", "latency/result_overwritten", "latency/accepted_action_seq",
            "latency/action_accept_ns", "latency/pd_sequence", "latency/pd_action_seq", "latency/pd_start_ns",
            "latency/pd_end_ns", "latency/first_pd_action_seq", "latency/first_pd_ns", "latency/replaced_before_pd")]
    else:
        joint = AXES.index(axis) < 4
        parameters = config["latency_probe"]["ros__parameters"]
        parameters.update(axis=axis, probe_amplitude_nm=amplitude if amplitude is not None else (1. if joint else .3),
                          torque_limit_nm=40. if joint else 4.5, joint_kd=4. if joint else .08,
                          speed_limit_rad_s=6. if joint else max(25., abs(wheel_velocity) + 10.),
                          wheel_velocity_rad_s=wheel_velocity, bit_duration_s=bit_duration,
                          segment_duration_s=segment_duration, joint_excursion_limit_rad=excursion)
        if (not all(math.isfinite(value) for value in (parameters["probe_amplitude_nm"], wheel_velocity, bit_duration, segment_duration, excursion))
                or not 0 < parameters["probe_amplitude_nm"] <= parameters["torque_limit_nm"] * .5
                or bit_duration < .002 or segment_duration < 2. or excursion <= 0):
            raise ValueError("Invalid excitation amplitude, duration, excursion or velocity")
        if joint and wheel_velocity != 0:
            raise ValueError("Wheel velocity only applies to wheel captures")
    for name in AXES:
        signals += [f"/wheel_leg/{name}/{suffix}" for suffix in (
            "angle", "velocity", "torque", "control_torque", "tau_frame_api", "tx_kind", "tx_sequence",
            "tx_queued_steady_ns", "feedback_sequence", "feedback_steady_ns")]
        signals += [f"/wheel_leg/{name}/{suffix}" for suffix in
                    (("temperature_mos", "temperature_rotor", "status_code", "fault_code")
                     if "joint" in name else ("temperature",))]
    signals += ["/wheel_leg/latency/trace_dropped", "/wheel_leg/latency/trace_failed"]
    collector["signals"] = list(dict.fromkeys(signals))
    source_paths = [root / path for path in (
        "rmcs_ws/src/rmcs_core/src/hardware/wheel_leg.cpp",
        "rmcs_ws/src/rmcs_core/src/hardware/device/actuator_latency_trace.hpp",
        "rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp",
        "rmcs_ws/src/rmcs_core/src/hardware/device/dji_motor.hpp",
        "rmcs_ws/src/rmcs_core/src/identification/wheel_leg_load_compensation.hpp",
        "rmcs_ws/src/rmcs_core/src/identification/wheel_leg_sweep_session.hpp",
        "rmcs_ws/src/rmcs_core/src/identification/wheel_leg_latency_plan.hpp",
        "rmcs_ws/src/rmcs_core/src/identification/wheel_leg_latency_probe_controller.cpp",
        "rmcs_ws/src/rmcs_rl/src/policy/policy_worker.hpp", "rmcs_ws/src/rmcs_rl/src/policy/policy_worker.cpp",
        "rmcs_ws/src/rmcs_rl/src/rl_bridge.cpp",
        "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg_rl_controller.cpp",
        "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg_observation.cpp")]
    source_paths += [config_path]
    for block in config.values():
        model_path = block.get("ros__parameters", {}).get("rl_model_path")
        if model_path:
            resolved = Path(model_path)
            if not resolved.is_absolute():
                resolved = root / "rmcs_ws/src/rmcs_rl" / resolved
            if not resolved.is_file():
                raise ValueError("Cannot freeze deployment model: " + str(resolved))
            source_paths.append(resolved)
    hashes = {str(path.relative_to(root)) if path.is_relative_to(root) else str(path): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in source_paths}
    output.mkdir(parents=True)
    parameters_path = output / "parameters.yaml"
    parameters_path.write_text(yaml.safe_dump(config, sort_keys=False))
    manifest = {"schema": "wheel_leg_latency_capture_v1", "axis": None if deployment else axis,
                "scope": "deployment" if deployment else "host_command_to_reported_current",
                "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
                "rl_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root / "rmcs_ws/src/rmcs_rl", text=True).strip(),
                "dirty": subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).splitlines(),
                "source_sha256": hashes, "parameters_sha256": hashlib.sha256(parameters_path.read_bytes()).hexdigest(),
                "unknown": ["firmware_identity", "usb_to_can_one_way", "can_to_current_foc_only", "telemetry_sensor_filter_delay"]}
    manifest["binary_sha256"] = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in (root / "rmcs_ws/install/lib/librmcs_core.so", root / "rmcs_ws/install/lib/librmcs_rl.so") if path.exists()}
    for path in source_paths:
        if path.is_relative_to(root):
            saved = output / "source" / path.relative_to(root)
            saved.parent.mkdir(parents=True, exist_ok=True)
            saved.write_bytes(path.read_bytes())
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    command = "ros2 run rmcs_executor rmcs_executor --ros-args --params-file " + shlex.quote(str(parameters_path))
    (output / "launch-command.txt").write_text(command + "\n")
    return manifest, command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--axis", choices=AXES, default="left_wheel")
    parser.add_argument("--amplitude", type=float)
    parser.add_argument("--wheel-velocity", type=float, default=0.)
    parser.add_argument("--deployment", type=Path, help="Trace an existing standalone RL YAML, without adding a torque producer")
    parser.add_argument("--bit-duration", type=float, default=.01)
    parser.add_argument("--segment-duration", type=float, default=10.)
    parser.add_argument("--joint-excursion", type=float, default=.1)
    args = parser.parse_args()
    _, command = prepare(args.root, args.output, args.axis, args.amplitude, args.wheel_velocity, args.deployment,
                         args.bit_duration, args.segment_duration, args.joint_excursion)
    print(command)


if __name__ == "__main__":
    main()
