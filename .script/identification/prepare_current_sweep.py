#!/usr/bin/env python3
"""Generate synchronized slow/fast angle sweeps, preserving the vehicle ADRC."""
import argparse
import copy
import json
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = ROOT / "rmcs_ws/src/rmcs_bringup/config"
JOINTS = ("left_front", "left_back", "right_back", "right_front")
CONTROLLERS = ("lf_joint_controller", "lb_joint_controller", "rb_joint_controller", "rf_joint_controller")

def generate(source, condition):
    if condition not in ("suspended", "grounded"):
        raise ValueError("Invalid condition")
    hardware = [c for c in source["rmcs_executor"]["ros__parameters"]["components"]
                if "rmcs_core::hardware::DeformableInfantryOmni" in c]
    if len(hardware) != 1:
        raise ValueError("Require one deformable hardware plugin")
    name = hardware[0].split("->")[1].strip()
    parameters = copy.deepcopy(source[name]["ros__parameters"])
    parameters.update(supercap_disabled=True, joint_current_command_limit=2048.0)
    limits = source["chassis_controller"]["ros__parameters"]
    components = hardware + ["rmcs_core::controller::identification::DeformableCurrentSweep -> current_sweep"]
    components += [f"rmcs_core::controller::chassis::DeformableJointController -> {c}" for c in CONTROLLERS]
    components += ["rmcs_core::debug::ValueCollector -> value_collector"]
    result = {"rmcs_executor": {"ros__parameters": {"update_rate": 1000.0, "components": components}},
              name: {"ros__parameters": parameters},
              "current_sweep": {"ros__parameters": {
                  "condition": condition, "min_angle_deg": float(limits["min_angle"]),
                  "max_angle_deg": float(limits["max_angle"]), "move_speed_rad_s": 0.08,
                  "slow_duration_s": 120.0, "fast_duration_s": 60.0,
                  "slow_start_frequency_hz": 0.02, "slow_end_frequency_hz": 0.06,
                   "fast_start_frequency_hz": 0.06, "fast_end_frequency_hz": 0.12,
                   "ultra_fast_frequency_hz": 1.0, "ultra_fast_duration_s": 10.0}}}
    signals = ["/identification/" + n for n in ("host_time_s", "elapsed_s", "run", "stage",
               "frequency_hz", "actuation_active", "remote_left", "remote_right")]
    for controller, joint in zip(CONTROLLERS, JOINTS):
        p = copy.deepcopy(source[controller]["ros__parameters"])
        prefix = f"/chassis/{joint}_joint"
        p.update(control=prefix + "/servo_current_raw", setpoint_angle=prefix + "/target_physical_angle",
                 setpoint_velocity=prefix + "/target_physical_velocity")
        result[controller] = {"ros__parameters": p}
        signals += [prefix + "/" + s for s in (
            "physical_angle", "physical_velocity", "rl_angle", "rl_velocity", "torque", "current_raw", "raw_current",
            "torque_per_current_raw", "target_torque_proxy", "command_torque_proxy", "physical_torque_proxy",
            "target_physical_torque_proxy", "command_physical_torque_proxy",
            "temperature", "target_physical_angle", "target_physical_velocity", "servo_current_raw",
            "control_current_raw", "command_current_raw", "command_sequence", "command_prepare_time_s",
            "command_limited", "feedback_sequence", "feedback_time_s", "feedback_age_s")]
    signals += ["/chassis/imu/roll", "/chassis/imu/pitch"]
    result["value_collector"] = {"ros__parameters": {
        "experiment_mode": True, "csv_path": f"/tmp/deformable_angle_sweep_{condition}_<timestamp>.csv",
        "signals": signals, "write_interval": 5, "flush_interval": 1000}}
    result["current_sweep"]["ros__parameters"]["experiment_config_json"] = json.dumps({
        "hardware": parameters,
        "adrc": {c: result[c]["ros__parameters"] for c in CONTROLLERS},
        "sweep": copy.deepcopy(result["current_sweep"]["ros__parameters"]),
        "collector": result["value_collector"]["ros__parameters"],
    }, separators=(",", ":"))
    return result

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=CONFIG_DIR / "deformable-infantry-omni-rl.yaml")
    parser.add_argument("--condition", choices=("suspended", "grounded"), required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or CONFIG_DIR / f"deformable-current-sweep-{args.condition}.yaml"
    output.write_text("# Generated synchronized angle sweep; DR16 double middle starts.\n"
                      + yaml.safe_dump(generate(yaml.safe_load(args.source.read_text()), args.condition), sort_keys=False))
    print(output)

if __name__ == "__main__":
    main()
