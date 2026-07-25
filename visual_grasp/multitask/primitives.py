"""Task primitives: small, reusable, testable robot actions.

Built on the existing horizontal side-grasp + MuJoCo Jacobian IK from sim/grasp_sim.py.
Each primitive returns a structured result dict {success, message, data} so the
executor (and a future bridge/LLM caller) can log and branch on every step.
"""
import os
import pathlib
import sys
from dataclasses import asdict, dataclass

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation, Slerp

os.environ.setdefault("MUJOCO_GL", "egl")

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import grasp_sim  # noqa: E402  (horizontal_grasp_frame, ik_target, GRIP_*)
from .control import MotionSafetyError, TrajectoryExecutionConfig


@dataclass
class GraspExecutionTrace:
    candidate_id: str
    direction_deg: float
    pre_close_object_xy_shift_mm: float
    close_object_xy_shift_mm: float
    lift_horizontal_drift_mm: float
    lift_height_mm: float
    target_contact_geoms: tuple
    bilateral_finger_contact: bool
    failure_class: str


def result(success, message, **data):
    return {"success": success, "message": message, "data": data}


def plan_horizontal_grasp(center_base, cfg):
    """Geometric grasp plan from an object center (no ground truth).

    Returns (grasp, grasp_mat, pre, lift). The grasp z is clamped to the table
    surface + clearance instead of the object's true z, so the plan uses only the
    camera-derived center plus the known table height.
    """
    grasp = np.asarray(center_base, dtype=float).copy()
    floor = cfg["scene"]["table_top_z"] + cfg["grasp"]["min_grasp_z_clearance"]
    if cfg["grasp"].get("use_table_grasp_z"):
        # Horizontal side grasp of a table object: the grasp height is a stable
        # table-relative value, not the noisy perception z (a few mm of vertical
        # jitter shifts the grasp into a worse arm config -- see DEVLOG IK unify).
        grasp[2] = floor
    else:
        grasp[2] = max(grasp[2], floor)
    grasp_mat = grasp_sim.horizontal_grasp_frame(grasp)
    pre = grasp_sim.horizontal_pregrasp_point(
        grasp, grasp_mat, distance=cfg["grasp"]["approach_distance"]
    )
    lift = grasp + np.array([0.0, 0.0, cfg["grasp"]["lift_height"]])
    return grasp, grasp_mat, pre, lift


def move_to(world, pos, mat=None, grip=None, steps=700,
            pos_tol=0.004, max_settle_steps=3000):
    """Solve IK to a target and drive the arm there. Fails loudly if IK diverges.

    A fixed settle-step budget alone does not guarantee the position actuators
    actually reach ``q`` -- how far the PD controller lags behind the target at
    that point depends on the arm's prior pose/momentum, not just on the target
    itself. For a horizontal pinch grasp a few mm of leftover tracking error is
    enough to close off-center and drop the object, while ``ik_target`` having
    "converged" (found *a* solution) says nothing about whether the arm
    actually got there. So keep settling in bounded extra batches until the
    TCP is actually within ``pos_tol`` of the target, and fail loudly (instead
    of silently proceeding to close the gripper) if it never does.
    """
    q, ok = grasp_sim.ik_target(world.model, world.data, world.tcp, pos, mat,
                                use_analytic=world.use_analytic_ik)
    if not ok:
        return result(False, f"IK did not converge for target {np.round(pos, 3)}")
    world.set_arm(q, grip, steps)
    total = steps
    while (np.linalg.norm(world.tcp_pos() - pos) > pos_tol
           and total < max_settle_steps):
        batch = min(200, max_settle_steps - total)
        world.settle(batch)
        total += batch
    err = float(np.linalg.norm(world.tcp_pos() - pos))
    if err > pos_tol:
        return result(False,
                      f"settled {err * 1000:.1f}mm from target {np.round(pos, 3)} "
                      f"after {total} steps (never converged)", tcp=world.tcp_pos())
    return result(True, "moved", tcp=world.tcp_pos(), settle_steps=total)


def open_gripper(world, steps=400):
    world.gripper(grasp_sim.GRIP_OPEN, steps)
    return result(True, "gripper opened")


def close_gripper(world, steps=600):
    world.gripper(grasp_sim.GRIP_CLOSE, steps)
    return result(True, "gripper closed", gripper_qpos=float(world.data.qpos[6]))


def execute_grasp(world, grasp, grasp_mat, pre, lift, solutions=None):
    """Run pre-grasp -> descend -> close -> lift. Returns the first failure or success.

    The pre-grasp approach is position-only (orientation free) and the horizontal
    gripper frame is only enforced at the grasp/lift, matching the validated
    sim/grasp_sim_camera.py sequence -- constraining the approach orientation makes
    the arm clip the object on the way in.

    The pre-approach and lift legs go through carry_to (Cartesian-interpolated)
    only on the analytic-IK (piper_real) model. A raw single jump's PD-driven
    path is uncontrolled and can sweep the still-open gripper through the
    object before the grasp starts, or fling a held object out on the way up
    (see DEVLOG piper_real place_into bug) -- confirmed root cause on
    piper_real. But a straight Cartesian line is not a universally safe
    replacement: on the menagerie model's home pose, a straight line down to
    `pre` clips the object from a *different* direction than the joint jump
    did, so forcing carry_to there regressed a previously-working path
    (caught by re-running the real end-to-end demo, not just an isolated
    repro -- see DEVLOG). Until there's a real collision-aware approach path,
    keep the single-jump behavior on menagerie, which is verified working.
    """
    if solutions:
        world.set_arm(solutions["pre"], grasp_sim.GRIP_OPEN, 700)
        r = result(True, "moved to planned pre-grasp")
    elif world.use_analytic_ik:
        r = carry_to(world, pre, None, grip=grasp_sim.GRIP_OPEN)
    else:
        r = move_to(world, pre, None, grasp_sim.GRIP_OPEN, steps=700)
    if not r["success"]:
        return result(False, "pre-grasp: " + r["message"])
    if solutions:
        world.set_arm(solutions["grasp"], grasp_sim.GRIP_OPEN, 700)
        r = result(True, "moved to planned grasp")
    else:
        r = move_to(world, grasp, grasp_mat, grasp_sim.GRIP_OPEN, steps=700)
    if not r["success"]:
        return result(False, "descend: " + r["message"])
    close_gripper(world)
    if solutions:
        world.set_arm(solutions["lift"], steps=900)
        r = result(True, "moved to planned lift")
    elif world.use_analytic_ik:
        r = carry_to(world, lift, grasp_mat)
    else:
        r = move_to(world, lift, grasp_mat, steps=900)
    if not r["success"]:
        return result(False, "lift: " + r["message"])
    return result(True, "grasp executed", tcp=world.tcp_pos())


def _target_contacts(world, target_body_name):
    target_id = mujoco.mj_name2id(
        world.model, mujoco.mjtObj.mjOBJ_BODY, target_body_name)
    contacts, finger_bodies = [], set()
    if target_id < 0:
        return (), False
    for contact in world.data.contact:
        geoms = (int(contact.geom1), int(contact.geom2))
        bodies = tuple(int(world.model.geom_bodyid[geom]) for geom in geoms)
        if target_id not in bodies:
            continue
        other_index = 1 if bodies[0] == target_id else 0
        other_geom, other_body = geoms[other_index], bodies[other_index]
        geom_name = mujoco.mj_id2name(
            world.model, mujoco.mjtObj.mjOBJ_GEOM, other_geom) or f"geom:{other_geom}"
        body_name = mujoco.mj_id2name(
            world.model, mujoco.mjtObj.mjOBJ_BODY, other_body) or f"body:{other_body}"
        contacts.append(f"{body_name}/{geom_name}")
        if body_name in ("link7", "link8"):
            finger_bodies.add(body_name)
    return tuple(sorted(set(contacts))), finger_bodies == {"link7", "link8"}


def _execution_trace(initial_pos, pre_close_pos, after_close_pos, final_pos,
                     contacts, bilateral, lift_threshold_m,
                     candidate_id, direction_deg):
    pre_shift = float(np.linalg.norm(pre_close_pos[:2] - initial_pos[:2]))
    close_shift = float(np.linalg.norm(after_close_pos[:2] - pre_close_pos[:2]))
    lift_drift = float(np.linalg.norm(final_pos[:2] - after_close_pos[:2]))
    lift_height = float(final_pos[2] - initial_pos[2])
    if pre_shift > 0.005:
        failure_class = "EARLY_APPROACH_CONTACT"
    elif not bilateral:
        failure_class = "OFF_CENTER_CLOSURE"
    elif lift_height <= lift_threshold_m and lift_drift > 0.005:
        failure_class = "SLIP_DURING_LIFT"
    elif lift_height <= lift_threshold_m:
        failure_class = "PATH_OR_IK_FAILURE"
    else:
        failure_class = "NONE"
    return GraspExecutionTrace(
        candidate_id=candidate_id, direction_deg=float(direction_deg),
        pre_close_object_xy_shift_mm=round(pre_shift * 1000.0, 3),
        close_object_xy_shift_mm=round(close_shift * 1000.0, 3),
        lift_horizontal_drift_mm=round(lift_drift * 1000.0, 3),
        lift_height_mm=round(lift_height * 1000.0, 3),
        target_contact_geoms=contacts, bilateral_finger_contact=bilateral,
        failure_class=failure_class)


def execute_motion_plan(world, motion_plan, target_body_name=None,
                        lift_threshold_m=0.03, candidate_id="", direction_deg=0.0):
    """Execute exactly the validated joint waypoints; never re-solve IK."""
    executed = []
    initial_pos = (world.body_pos(target_body_name).copy()
                   if target_body_name is not None else None)
    pre_close_pos = initial_pos.copy() if initial_pos is not None else None
    after_close_pos = initial_pos.copy() if initial_pos is not None else None
    contacts, bilateral = (), False
    for segment in motion_plan.segments:
        if segment.phase == "LIFT":
            if target_body_name is not None:
                pre_close_pos = world.body_pos(target_body_name).copy()
            close_gripper(world)
            if target_body_name is not None:
                after_close_pos = world.body_pos(target_body_name).copy()
                contacts, bilateral = _target_contacts(world, target_body_name)
        grip = grasp_sim.GRIP_CLOSE if segment.phase == "LIFT" else grasp_sim.GRIP_OPEN
        # First waypoint equals the previous segment endpoint (or validated start_q).
        for waypoint in segment.waypoints[1:]:
            world.set_arm(np.asarray(waypoint.q, float), grip, steps=700)
            executed.append({"segment": segment.name, "waypoint": waypoint.name,
                             "q": np.asarray(waypoint.q, float).copy()})
    trace = None
    if target_body_name is not None:
        trace = _execution_trace(
            initial_pos, pre_close_pos, after_close_pos,
            world.body_pos(target_body_name).copy(), contacts, bilateral,
            lift_threshold_m, candidate_id, direction_deg)
    return result(True, "validated motion plan executed", tcp=world.tcp_pos(),
                  executed_waypoints=executed,
                  execution_trace=asdict(trace) if trace is not None else None)


def execute_continuous_motion_plan(
        world, motion_plan, config: TrajectoryExecutionConfig,
        target_body_name=None, lift_threshold_m=0.03,
        candidate_id="", direction_deg=0.0):
    """Execute validated waypoints as continuous, velocity-limited polylines."""
    if config.control_period_steps != 1:
        return result(False, "only one-step control periods are supported",
                      reason="UNSUPPORTED_CONTROL_PERIOD")
    initial_pos = (world.body_pos(target_body_name).copy()
                   if target_body_name is not None else None)
    pre_close_pos = initial_pos.copy() if initial_pos is not None else None
    after_close_pos = initial_pos.copy() if initial_pos is not None else None
    contacts, bilateral = (), False
    executed, reports = [], []
    try:
        for segment in motion_plan.segments:
            if segment.phase == "LIFT":
                if target_body_name is not None:
                    pre_close_pos = world.body_pos(target_body_name).copy()
                close_gripper(world, steps=config.gripper_settle_steps)
                if target_body_name is not None:
                    after_close_pos = world.body_pos(target_body_name).copy()
                    contacts, bilateral = _target_contacts(world, target_body_name)
            grip = (grasp_sim.GRIP_CLOSE if segment.phase == "LIFT"
                    else grasp_sim.GRIP_OPEN)
            settle_steps = (config.final_settle_steps
                            if segment.phase in ("FINAL_APPROACH", "LIFT") else 0)
            report = world.follow_joint_waypoints(
                [waypoint.q for waypoint in segment.waypoints], grip=grip,
                max_joint_velocity_rad_s=config.max_joint_velocity_rad_s,
                max_following_error_rad=config.max_following_error_rad,
                final_settle_steps=settle_steps, phase=segment.phase,
                min_table_clearance_m=config.min_table_clearance_m)
            reports.append({
                "segment": segment.name, "steps": report.steps,
                "max_following_error_rad": report.max_following_error_rad,
                "min_table_clearance_mm": (
                    round(report.min_table_clearance_m * 1000.0, 3)
                    if report.min_table_clearance_m is not None else None),
            })
            executed.extend({"segment": segment.name, "waypoint": waypoint.name,
                             "q": np.asarray(waypoint.q, float).copy()}
                            for waypoint in segment.waypoints[1:])
    except MotionSafetyError as exc:
        return result(False, str(exc), reason="MOTION_SAFETY_GUARD",
                      trajectory_reports=reports)
    trace = None
    if target_body_name is not None:
        trace = _execution_trace(
            initial_pos, pre_close_pos, after_close_pos,
            world.body_pos(target_body_name).copy(), contacts, bilateral,
            lift_threshold_m, candidate_id, direction_deg)
    return result(True, "continuous validated motion plan executed",
                  tcp=world.tcp_pos(), executed_waypoints=executed,
                  trajectory_reports=reports,
                  simulated_motion_s=round(
                      (sum(row["steps"] for row in reports) +
                       config.gripper_settle_steps) *
                      float(world.model.opt.timestep), 4),
                  execution_trace=asdict(trace) if trace is not None else None)


def pick_object(world, obj, cfg):
    """Convenience: plan + execute a horizontal grasp for an object record."""
    grasp, grasp_mat, pre, lift = plan_horizontal_grasp(obj["center_base"], cfg)
    return execute_grasp(world, grasp, grasp_mat, pre, lift)


def carry_to(world, target, mat, n_waypoints=6, steps=200, grip=None):
    """Move the TCP to target through interpolated Cartesian waypoints, holding
    ``grip`` fixed (defaults to closed, for carrying a held object). A single
    large jump snaps the arm through whatever lies in between -- for a closed
    grip that flings the held object out, and for an open grip (the pre-grasp
    approach) it sweeps the fingers straight through the target object before
    the grasp sequence even starts, which can pin the finger joints past their
    own range and never release. Splitting the path into small Cartesian steps
    keeps the arm close to a straight line and avoids both."""
    if grip is None:
        grip = grasp_sim.GRIP_CLOSE
    start = world.tcp_pos()
    target = np.asarray(target, dtype=float)
    for i in range(1, n_waypoints + 1):
        wp = start + (target - start) * (i / n_waypoints)
        q, ok = grasp_sim.ik_target(world.model, world.data, world.tcp, wp, mat,
                                    use_analytic=world.use_analytic_ik)
        if not ok:
            return result(False, f"carry IK diverged at waypoint {i}/{n_waypoints}")
        world.set_arm(q, grip, steps)
    return result(True, "carried", tcp=world.tcp_pos())


def release_at(world, point_base, z_offset, n_waypoints=9,
                max_joint_velocity_rad_s=0.5):
    """Place the held object: carry above the point (gripper held horizontal so the
    object does not tip out), open, then retreat.

    The carry is two legs -- horizontal at a safe height, then straight down --
    not one diagonal line. A single diagonal carry_to from wherever the lift
    left off to `above` clips whatever sits at an intermediate height along
    that line: verified on piper_real place_into, the held object hit the
    target bowl's rim partway through the diagonal and was knocked loose
    before the gripper ever opened, yet still landed close enough afterward to
    pass the xy-only VERIFY_DONE check (see DEVLOG).

    The horizontal leg also SLERPs the gripper orientation from "however the
    object is actually being held" to the destination-facing frame across all
    waypoints, instead of snapping to it in one step or never reorienting (the
    latter breaks far-off-heading place_at targets, where the pick orientation
    isn't a reachable/sane pose at the destination). But a smooth CARTESIAN
    orientation sweep does not guarantee a smooth JOINT-space path: a 2-finger
    gripper's closing axis is a line, so two wrist rolls (0 and pi apart) are
    equally valid for "the same" grasp orientation, and which one is analytic
    IK-reachable can flip partway through the sweep -- verified on piper_real
    this is a genuine kinematic transition (locking one roll for the whole
    sequence makes the later waypoints unsolvable), not just seed noise.
    Forcing that transition through in a single settle snaps the wrist ~180
    degrees and flings the held object. So the whole transit+descend is
    solved as one joint-space plan up front, then executed through
    follow_joint_waypoints, which velocity-limits however large that
    per-waypoint jump turns out to be instead of snapping to it.
    """
    above = np.asarray(point_base, dtype=float) + np.array([0.0, 0.0, z_offset])
    carry_mat = np.array(world.data.site_xmat[world.tcp]).reshape(3, 3).copy()
    target_mat = grasp_sim.horizontal_grasp_frame(above)
    transit = np.array([above[0], above[1],
                        max(float(world.tcp_pos()[2]), float(above[2]))])
    slerp = Slerp([0.0, 1.0], Rotation.from_matrix([carry_mat, target_mat]))
    start = world.tcp_pos().copy()
    cartesian = ([(start + (transit - start) * (i / n_waypoints),
                   slerp(i / n_waypoints).as_matrix())
                  for i in range(1, n_waypoints + 1)] +
                 [(above, target_mat)])
    q_waypoints = [np.array(world.data.qpos[:6], float)]
    for i, (wp, wp_mat) in enumerate(cartesian, start=1):
        q, ok = grasp_sim.ik_target(world.model, world.data, world.tcp, wp, wp_mat,
                                    use_analytic=world.use_analytic_ik)
        if not ok:
            return result(False, f"carry-to-place IK diverged at waypoint {i}")
        q_waypoints.append(np.asarray(q, float))
    try:
        world.follow_joint_waypoints(
            q_waypoints, grip=grasp_sim.GRIP_CLOSE,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s, phase="PLACE_TRANSIT")
    except MotionSafetyError as exc:
        return result(False, f"carry-to-place: {exc}")
    open_gripper(world)
    r = move_to(world, above + np.array([0.0, 0.0, 0.05]), None, steps=500)
    if not r["success"]:
        return result(False, "retreat: " + r["message"])
    return result(True, "released", tcp=world.tcp_pos())
