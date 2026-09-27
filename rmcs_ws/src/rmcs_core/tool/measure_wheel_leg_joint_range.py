#!/usr/bin/env python3
"""Measure V5 motor-pair constraints from PhysX joint-axis positions.

The user supplies the physical knee range. This determines the corresponding
motor difference; a simulation cannot discover the real vehicle's hard stops.
The knee angle is measured at the knee, in its hinge plane, with no angle LUT.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--bundle", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--device", default="cpu")
parser.add_argument("--motor-poses", type=Path,
                    help="JSON list of {name, angles_rad:[LH,LK,RH,RK]}; measure these poses instead of the range sweep")
args = parser.parse_args()

import numpy as np
from scipy.optimize import brentq

spec = json.loads((args.bundle / "model_spec.json").read_text())
manifest = json.loads((args.bundle / "manifest.json").read_text())
sys.path.insert(0, str(args.bundle / "tools"))
from build_v5_closedchain import solve_pose
from v5_mechanism import fk

active = ["L_joint1", "LL_joint1", "R_joint1", "RR_joint1"]
offsets = np.array([-1.6, -2.93, 1.6, 2.93])
axes = {s: np.asarray(next(j["axis"] for j in spec["joints"]
                          if j["child"] == s + "_link2")) for s in ("L", "R")}


def angle_from_axes(hip, knee, wheel, normal, project=True):
    thigh, shank = hip - knee, wheel - knee
    if project:
        thigh = thigh - np.sum(thigh * normal, axis=-1, keepdims=True) * normal
        shank = shank - np.sum(shank * normal, axis=-1, keepdims=True) * normal
    cosine = np.sum(thigh * shank, axis=-1) / (np.linalg.norm(thigh, axis=-1)
                                             * np.linalg.norm(shank, axis=-1))
    return np.degrees(np.arccos(np.clip(cosine, -1., 1.)))


def kinematic_angle(q, side):
    f = fk(spec, q)
    return float(angle_from_axes(*(f[side + "_link" + str(n)][:3, 3] for n in (1, 2, 3)),
                                 f[side + "_link2"][:3, :3] @ axes[side]))


def closed_at_angle(angle, hip):
    seed = dict(spec["nominal_joint_pos"])
    for name in active[:2]:
        seed[name] += hip - .42
    for name in active[2:]:
        seed[name] -= hip - .42

    def pose(knee):
        return solve_pose(spec, {"L_joint1": hip, "L_joint2": knee, "L_joint3": 0.,
                                 "R_joint1": -hip, "R_jonit2": -knee, "R_joint3": 0.}, seed)

    knee = brentq(lambda x: kinematic_angle(pose(x), "L") - angle, -.2, 1.3, xtol=1e-12)
    return pose(knee)


cases = []
poses = []
if args.motor_poses:
    requested = json.loads(args.motor_poses.read_text())
    motor_poses = []
    for entry in requested:
        values = np.asarray(entry["angles_rad"], dtype=float)
        if values.shape != (4,) or not np.all(np.isfinite(values)):
            raise ValueError("Each motor pose requires four finite radians in LH,LK,RH,RK order")
        motor_poses.append((entry["name"], values))
    if not motor_poses:
        raise ValueError("At least one motor pose is required")
else:
    for heading in (-1.6, 0., 2.6):
        for angle in range(45, 106, 5):
            q = closed_at_angle(angle, heading)
            cases.append({"kind": "range_sample", "prescribed_inner_deg": angle,
                          "prescribed_left_hip_rad": heading})
            poses.append(q)
    motor_poses = (("urdf_zero", np.zeros(4)), ("calibration", offsets),
                   ("reported_stop", np.deg2rad([3.03, -4.39, 11.72, -27.02])))
for label, values in motor_poses:
    q = solve_pose(spec, dict(zip(active, values)) | {"L_joint3": 0., "R_joint3": 0.},
                   dict(spec["nominal_joint_pos"]))
    cases.append({"kind": label})
    poses.append(q)

from isaaclab.app import AppLauncher
launcher = AppLauncher({"headless": True, "device": args.device})
try:
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.actuators import IdealPDActuatorCfg
    from pxr import UsdGeom, UsdPhysics

    torch.set_num_threads(4)
    dt = .001
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
        device=args.device, dt=dt, gravity=(0., 0., 0.), render_interval=100))
    for i in range(len(cases)):
        UsdGeom.Xform.Define(sim.stage, f"/World/envs/env_{i}").AddTranslateOp().Set(
            (i % 7 * 2., i // 7 * 2., 0.))
    passive = [n for n in manifest["tree_joint_names"] if n not in active]
    robot = Articulation(ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=str(args.bundle / "robot.usda"),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                fix_root_link=True, enabled_self_collisions=False,
                solver_position_iteration_count=32, solver_velocity_iteration_count=8)),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0., 0., 1.),
            joint_pos=manifest["nominal_joint_pos"], joint_vel={".*": 0.}),
        actuators={
            "motors": IdealPDActuatorCfg(joint_names_expr=active, stiffness=400., damping=10.,
                effort_limit=40., effort_limit_sim=40., velocity_limit_sim=45., armature=.02),
            "passive": IdealPDActuatorCfg(joint_names_expr=passive, stiffness=0., damping=.002,
                effort_limit=1000., effort_limit_sim=1000., velocity_limit_sim=1000.)},
        soft_joint_pos_limit_factor=1.0))
    loops = [p for p in sim.stage.Traverse() if p.IsA(UsdPhysics.SphericalJoint)]
    assert len(loops) == 6 * len(cases)
    # Remove research-asset hard stops so they cannot force the measured range.
    # All six loop constraints stay active. No changes are saved to the asset.
    for prim in sim.stage.Traverse():
        if prim.GetName() in ("L_joint2", "R_jonit2"):
            joint = UsdPhysics.RevoluteJoint(prim)
            joint.GetLowerLimitAttr().Set(-180.)
            joint.GetUpperLimitAttr().Set(180.)
        if prim.GetName() in manifest["spring_joint_names"]:
            joint = UsdPhysics.PrismaticJoint(prim)
            joint.GetLowerLimitAttr().Set(-.1)
            joint.GetUpperLimitAttr().Set(.2)
    sim.reset()
    ids = [robot.joint_names.index(n) for n in active]
    initial = torch.tensor([[q[n] for n in robot.joint_names] for q in poses],
                           dtype=torch.float32, device=args.device)
    robot.write_joint_state_to_sim(initial, torch.zeros_like(initial))
    robot.reset()
    robot.set_joint_position_target(initial[:, ids], joint_ids=ids)
    for _ in range(1000):
        robot.write_data_to_sim()
        sim.step(render=False)
        robot.update(dt)

    bodies = robot.data.body_link_pose_w.cpu().numpy().astype(np.float64)

    def rotated(body, local):
        pose = bodies[:, robot.body_names.index(body)]
        vec = np.broadcast_to(local, (len(cases), 3))
        cross = np.cross(pose[:, 4:7], vec)
        return vec + 2. * (pose[:, 3:4] * cross + np.cross(pose[:, 4:7], cross))

    def world(body, local):
        return bodies[:, robot.body_names.index(body), :3] + rotated(body, local)

    measured = robot.data.joint_pos[:, ids].cpu().numpy().astype(np.float64)
    inner = np.column_stack([angle_from_axes(
        *(world(s + "_link" + str(n), [0., 0., 0.]) for n in (1, 2, 3)),
        rotated(s + "_link2", axes[s])) for s in ("L", "R")])
    spatial = np.column_stack([angle_from_axes(
        *(world(s + "_link" + str(n), [0., 0., 0.]) for n in (1, 2, 3)),
        rotated(s + "_link2", axes[s]), project=False) for s in ("L", "R")])
    difference = (np.column_stack((measured[:, 0] - measured[:, 1],
                                  measured[:, 3] - measured[:, 2])) + np.pi) % (2*np.pi) - np.pi
    gap = np.max(np.stack([np.linalg.norm(world(c["body0"], c["local_pos0_m"])
                                         - world(c["body1"], c["local_pos1_m"]), axis=1)
                           for c in spec["constraints"]]), axis=0)
    root = world("base_link", [0., 0., 0.])
    axis_points = {s: {key: world(s + "_link" + str(n), [0., 0., 0.]) - root
                      for key, n in (("hip", 1), ("knee", 2), ("wheel", 3))}
                   for s in ("L", "R")}
    for i, case in enumerate(cases):
        case.update(inner_from_axes_deg=inner[i].tolist(), spatial_angle_deg=spatial[i].tolist(),
                    motor_angles_rad=measured[i].tolist(), difference_rad=difference[i].tolist(),
                    max_loop_gap_m=float(gap[i]),
                    joint_axes_relative_to_base_m={
                        s: {key: value[i].tolist() for key, value in points.items()}
                        for s, points in axis_points.items()},
                    within_nominal_motor_range=[bool(-1e-5 <= d <= 1.33 + 1e-5)
                                                for d in difference[i]],
                    kinematic_inner_deg=[kinematic_angle(poses[i], s) for s in ("L", "R")])
    sampled = [c for c in cases if c["kind"] == "range_sample"]
    max_angle_error = max(abs(c["inner_from_axes_deg"][s]
                              - (c["prescribed_inner_deg"] if sampled else c["kinematic_inner_deg"][s]))
                          for c in (sampled or cases) for s in range(2))
    bounds_difference = difference[:len(sampled)] if sampled else difference
    report = {
        "method": f"{len(cases)} PhysX closed-chain instances, 19 bodies / 6 live loops each; measure H-K and W-K projected into the passive knee hinge plane from body_link_pose_w",
        "mode": "supplied_motor_poses" if args.motor_poses else "physical_range_mapping",
        "physical_range_given_by_user_deg": [45., 105.],
        "physical_range_is_simulation_discovery": False,
        "nominal_motor_range_numerical_tolerance_rad": 1e-5,
        "fixture": "fixed base, gravity/gas spring off, only four active motors hold each sample for 1 s, passive joints free",
        "passive_state_writes_after_reset": 0, "solver_calls_during_physics": 0,
        "source_hashes": {n: hashlib.sha256((args.bundle / n).read_bytes()).hexdigest()
                          for n in ("source_urdf_v5.0.urdf", "model_spec.json", "robot.usda")},
        "difference_bounds_rad": [np.min(bounds_difference, axis=0).tolist(),
                                  np.max(bounds_difference, axis=0).tolist()],
        "max_angle_error_deg": max_angle_error, "max_loop_gap_m": float(gap.max()),
        "cases": cases,
        "passed_checks": "PhysX vs kinematics angle agreement and closed-loop gaps; not real hardware calibration or admissibility",
        "passed": bool(max_angle_error < .05 and gap.max() < .0001)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("JOINT_RANGE_RESULT", json.dumps({k: report[k] for k in
          ("passed", "difference_bounds_rad", "max_angle_error_deg", "max_loop_gap_m")}), flush=True)
    assert report["passed"], "Inspect range measurement report"
finally:
    if "sim" in locals():
        sim_utils.SimulationContext.clear_instance()
    launcher.app.close(wait_for_replicator=False)
