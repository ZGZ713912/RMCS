#!/usr/bin/env python3
"""Freeze a leg experiment configuration and source hashes without starting hardware."""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

import yaml


def prepare(root, project, side, fixture, output):
    root, output = root.resolve(), output.resolve()
    config_dir = root / "rmcs_ws/src/rmcs_bringup/config"
    paths = [config_dir / "wheel-leg-joints-sweep.yaml", config_dir / f"wheel-leg-joints-{project}-sweep.yaml"]
    parameters = {}
    for path in paths:
        for node, block in yaml.safe_load(path.read_text()).items():
            parameters.setdefault(node, {"ros__parameters": {}})["ros__parameters"].update(copy.deepcopy(block["ros__parameters"]))
    parameters["frequency_sweep"]["ros__parameters"].update(side=side, joint_fixture=fixture)
    parameters["value_collector"]["ros__parameters"]["csv_path"] = str(output / "raw.csv")
    names = ["rmcs_ws/src/rmcs_core/src/identification/" + name for name in (
        "wheel_leg_frequency_sweep_controller.cpp", "wheel_leg_sweep_plan.hpp", "wheel_leg_breakaway_probe.hpp",
        "wheel_leg_load_compensation.hpp", "wheel_leg_sweep_session.hpp", "wheel_leg_pair_recording_plan.hpp")]
    names += ["rmcs_ws/src/rmcs_core/src/controller/pid/position_velocity_p.hpp",
              "rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp"]
    source_bytes = {name: (root / name).read_bytes() for name in names}
    sources = {name: hashlib.sha256(value).hexdigest() for name, value in source_bytes.items()}
    library = root / "rmcs_ws/install/lib/librmcs_core.so"
    library_sha = hashlib.sha256(library.read_bytes()).hexdigest() if library.is_file() else None
    payload = json.dumps({"parameters": parameters, "source_sha256": sources, "built_library_sha256": library_sha}, sort_keys=True).encode()
    contract_sha = hashlib.sha256(payload).hexdigest()
    contract_id = int(contract_sha[:15], 16)
    parameters["frequency_sweep"]["ros__parameters"]["joint_contract_id"] = contract_id
    git_head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    git_status = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True)
    manifest = {"schema": "wheel_leg_collection_v1", "project": project, "side": side, "fixture": fixture,
        "excitation_revision": 8, "contract_sha256": contract_sha, "contract_id": contract_id,
        "source_head": git_head, "source_dirty": bool(git_status), "source_sha256": sources,
        "built_library_sha256": library_sha,
        "configuration_sha256": {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        "raw_csv": "raw.csv", "parameters": "parameters.yaml", "hardware_started": False,
        "fixture_declaration_requires_operator_check": True}
    parameter_text = yaml.safe_dump(parameters, sort_keys=False, allow_unicode=True)
    manifest["parameters_sha256"] = hashlib.sha256(parameter_text.encode()).hexdigest()
    output.mkdir(parents=True, exist_ok=False)
    (output / "parameters.yaml").write_text(parameter_text)
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    for name in names:
        destination = output / "source" / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(source_bytes[name])
    command = "ros2 run rmcs_executor rmcs_executor --ros-args --params-file " + shlex.quote(str(output / "parameters.yaml"))
    (output / "launch-command.txt").write_text(command + "\n")
    return manifest, command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", choices=("friction", "load"), required=True)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--fixture", choices=("horizontal", "vertical"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest, command = prepare(Path(__file__).resolve().parents[2], args.project, args.side, args.fixture, args.output)
    print(json.dumps({"manifest": str(args.output.resolve() / "manifest.json"),
                      "contract_id": manifest["contract_id"], "hardware_started": False}, ensure_ascii=False))
    print(command)


if __name__ == "__main__":
    main()
