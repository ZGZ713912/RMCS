#!/usr/bin/env python3
"""Wait for ValueCollector to finish, fetch logs over SSH, and summarize locally.

Does not start or move the robot. Run on the computer before explicitly starting
the experiment; already finished logs are also supported.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys
import time

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--remote", default="remote")
    parser.add_argument("--condition", choices=("suspended", "grounded", "all"), required=True)
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parents[2] / "data/identification")
    parser.add_argument("--timeout-s", type=float, default=7200)
    parser.add_argument("--poll-s", type=float, default=5)
    parser.add_argument("--skip-fit", action="store_true")
    parser.add_argument("--raw-only", action="store_true", help="Copy logs without requiring NumPy/SciPy")
    parser.add_argument("--watch", action="store_true", help="Keep collecting subsequent runs")
    args = parser.parse_args()
    if args.timeout_s <= 0 or args.poll_s <= 0 or args.remote.startswith("-"):
        parser.error("Positive timeout/poll interval and valid SSH host required")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    # Listing is deliberately remote: logs live on the robot, not in the workspace.
    condition = "*" if args.condition == "all" else args.condition
    script = ("import pathlib,json; "
              f"files=sorted(list(pathlib.Path('/tmp').glob('deformable_current_sweep_{condition}_*.csv.status.json'))+list(pathlib.Path('/tmp').glob('deformable_angle_sweep_{condition}_*.csv.status.json')),"
              "key=lambda p:p.stat().st_mtime,reverse=True); "
              "print(json.dumps([str(p)[:-12] for p in files]))")
    deadline = time.monotonic() + args.timeout_s
    while args.watch or time.monotonic() < deadline:
        query = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
                                args.remote, "python3 -c " + shlex.quote(script)],
                               capture_output=True, text=True)
        if query.returncode == 0:
            for remote_path in json.loads(query.stdout):
                local = args.output_dir / Path(remote_path).name
                if (args.raw_only and Path(str(local) + ".received").exists()) or local.with_suffix(".model.json").exists() or (
                    args.skip_fit and local.with_suffix(".static_hold.csv").exists()):
                    continue
                paths = [remote_path + suffix for suffix in ("", ".json", ".status.json")]
                # Status is copied last so a failed copy can safely be retried.
                for p in paths:
                    result = subprocess.run(["scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
                                             args.remote + ":" + p, str(args.output_dir)])
                    if result.returncode:
                        break
                else:
                    print(f"Collected: {local}", flush=True)
                    Path(str(local) + ".received").write_text(remote_path + "\n")
                    if json.loads(Path(str(local) + ".json").read_text()).get("command_domain") == "angle_reference":
                        from summarize_angle_sweep import summarize as summarize_angle
                        summarize_angle(local)
                        if args.watch:
                            continue
                        return 0
                    if args.raw_only:
                        if args.watch:
                            continue
                        return 0
                    from summarize_static_hold import summarize
                    try:
                        summarize(local, local.with_suffix(".static_hold.csv"))
                    except ValueError as error:
                        print(f"Static hold summary unavailable: {error}", file=sys.stderr)
                        if args.watch:
                            continue
                        return 2
                    if not args.skip_fit:
                        result = subprocess.run([sys.executable, str(Path(__file__).with_name("fit_current_sweep.py")),
                                               str(local), "--output", str(local.with_suffix(".model.json"))]).returncode
                        if not args.watch:
                            return result
                    elif not args.watch:
                        return 0
        else:
            print(query.stderr.strip(), file=sys.stderr)
        time.sleep(args.poll_s)
    print("Timed out waiting for a completed collector file", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
