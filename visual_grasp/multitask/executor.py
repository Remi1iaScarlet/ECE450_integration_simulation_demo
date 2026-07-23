"""Finite-state task executor.

Runs a task as an explicit sequence of states (TODO section 8) and returns a
structured, JSON-serializable result with a per-state log. Each state handler
mutates a shared ctx and returns {success, message, data}; the pipeline stops at
the first failure.

    pick:        DETECT_SOURCE -> PLAN_PICK -> EXECUTE_PICK -> VERIFY_GRASP
    place_at:    pick states   -> PLAN_PLACE -> EXECUTE_PLACE -> VERIFY_DONE
    place_into:  pick states   -> DETECT_TARGET -> PLAN_PLACE -> EXECUTE_PLACE -> VERIFY_DONE
    clear_table: place_into per source label

Headless demo:
    MUJOCO_GL=egl python3 -m multitask.executor --task pick --target cup
    MUJOCO_GL=egl python3 -m multitask.executor --task place_at --target cup --xyz 0.40,0.12,0.17
"""
import argparse
import json
import os
import pathlib
import time
import warnings

import numpy as np
import mujoco

os.environ.setdefault("MUJOCO_GL", "egl")

from . import grasp_planner, object_registry, primitives
from .config import load_config
from .world import SimWorld

ROOT = pathlib.Path(__file__).resolve().parent.parent
EVIDENCE_DIR = pathlib.Path(__file__).resolve().parent / "evidence"


def _native(obj):
    """Make numpy types JSON-serializable."""
    if isinstance(obj, np.ndarray):
        return [round(float(v), 4) for v in obj]
    if isinstance(obj, (np.floating, np.integer)):
        return round(float(obj), 4)
    if isinstance(obj, dict):
        return {k: _native(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_native(v) for v in obj]
    return obj


class TaskExecutor:
    """Drives one task to completion through a sequence of state handlers."""

    def __init__(self, world, cfg=None, verbose=True):
        self.world = world
        self.cfg = cfg or load_config()
        self.verbose = verbose
        self.log = []

    def _body_name(self, label):
        """MuJoCo body name for a YOLO label (for ground-truth verification)."""
        return self.cfg["objects"].get("body_of", {}).get(label, label)

    # On failure, these states loop back to an earlier state for a bounded retry
    # (e.g. a failed grasp verification re-attempts the grasp) instead of aborting.
    RETRY_TO = {"VERIFY_GRASP": "DETECT_SOURCE"}

    # ---- engine ------------------------------------------------------------
    def _run(self, task, target, states, ctx):
        ctx["_task_started_at"] = time.perf_counter()
        max_retries = self.cfg["grasp"].get("max_retries", 0)
        attempts = {}
        i = 0
        while i < len(states):
            name = states[i]
            state_started_at = time.perf_counter()
            res = getattr(self, f"_st_{name.lower()}")(ctx)
            duration_ms = (time.perf_counter() - state_started_at) * 1000.0
            reason = res.get("data", {}).get("reason")
            self.log.append({"state": name, "success": res["success"],
                             "message": res["message"],
                             "reason": reason or ("OK" if res["success"] else "STATE_FAILED"),
                             "duration_ms": round(duration_ms, 1)})
            if self.verbose:
                print(f"[{'ok ' if res['success'] else 'FAIL'}] {name}: {res['message']}")
            if res["success"]:
                i += 1
                continue
            back = self.RETRY_TO.get(name)
            if back in states and attempts.get(name, 0) < max_retries:
                attempts[name] = attempts.get(name, 0) + 1
                recovery_started = time.perf_counter()
                recovery = self._st_recover_pick(ctx)
                self.log.append({
                    "state": "RECOVER_PICK", "success": recovery["success"],
                    "message": recovery["message"],
                    "reason": recovery.get("data", {}).get(
                        "reason", "OK" if recovery["success"] else "STATE_FAILED"),
                    "duration_ms": round((time.perf_counter() - recovery_started) * 1000.0, 1),
                })
                if not recovery["success"]:
                    return self._summary(task, target, False, ctx)
                if self.verbose:
                    print(f"[retry] {name} failed -> re-running from {back} "
                          f"({attempts[name]}/{max_retries})")
                i = states.index(back)
                continue
            return self._summary(task, target, False, ctx)
        return self._summary(task, target, True, ctx)

    # ---- tasks -------------------------------------------------------------
    PICK_STATES = ["DETECT_SOURCE", "PLAN_PICK", "EXECUTE_PICK", "VERIFY_GRASP"]

    def pick(self, label, policy="nearest"):
        ctx = {"source_label": label, "policy": policy, "observation_version": 0}
        return self._run("pick", label, self.PICK_STATES, ctx)

    def place_at(self, label, xyz, policy="nearest"):
        ctx = {"source_label": label, "policy": policy, "observation_version": 0,
               "place_xyz": np.asarray(xyz, float), "place_kind": "surface"}
        states = self.PICK_STATES + ["PLAN_PLACE", "EXECUTE_PLACE", "VERIFY_DONE"]
        return self._run("place_at", label, states, ctx)

    def place_into(self, source_label, container_label=None, container_xyz=None,
                   policy="nearest"):
        ctx = {"source_label": source_label, "policy": policy, "observation_version": 0,
               "container_label": container_label, "place_kind": "container"}
        if container_xyz is not None:
            ctx["place_xyz"] = np.asarray(container_xyz, float)
        states = list(self.PICK_STATES)
        if "place_xyz" not in ctx:
            states.append("DETECT_TARGET")     # resolve container by vision
        states += ["PLAN_PLACE", "EXECUTE_PLACE", "VERIFY_DONE"]
        return self._run("place_into", source_label, states, ctx)

    def clear_table(self, labels, container_label=None, container_xyz=None,
                    policy="nearest"):
        results = []
        for label in labels:
            self.log = []  # fresh per-object log
            r = self.place_into(label, container_label, container_xyz, policy)
            results.append(r)
        ok = all(r["success"] for r in results)
        return _native({"task": "clear_table", "labels": labels,
                        "success": ok, "attempts": results})

    # ---- pick states -------------------------------------------------------
    def _st_detect_source(self, ctx):
        ctx["observation_version"] = int(ctx.get("observation_version", 0)) + 1
        def scan():
            return object_registry.scan(
                self.world, self.cfg["detector"]["scan_poses"],
                graspable=self.cfg["objects"]["graspable"],
                containers=self.cfg["objects"]["containers"],
                conf=self.cfg["detector"]["confidence_threshold"],
                table_z=self.cfg["scene"]["table_top_z"],
                backend=self.cfg["detector"].get("backend", "yolo"),
                body_of=self.cfg["objects"].get("body_of", {}),
                radius_factor=self.cfg["detector"].get("radius_correction", 1.0),
                motion_config=self.cfg.get("motion", {}))
        objs = scan()
        ctx["registry"] = objs
        ctx["n_detected"] = len(objs)
        obj = object_registry.get_object(objs, ctx["source_label"], ctx["policy"])
        if obj is None and ctx.get("awaiting_reobservation", False):
            self.log.append({"state": "CONFIRM_SOURCE", "success": False,
                             "message": "target not observed; running independent confirmation scan",
                             "reason": "TARGET_NOT_OBSERVED_AFTER_RECOVERY", "duration_ms": 0.0})
            objs = scan()
            ctx["registry"] = objs
            ctx["n_detected"] = len(objs)
            obj = object_registry.get_object(objs, ctx["source_label"], ctx["policy"])
            if obj is None:
                ctx.pop("awaiting_reobservation", None)
                return primitives.result(False, f"'{ctx['source_label']}' absent in both "
                                         "recovery scans",
                                         reason="TARGET_LOST_AFTER_RECOVERY")
            self.log.append({"state": "CONFIRM_SOURCE", "success": True,
                             "message": "target recovered by confirmation scan",
                             "reason": "OK", "duration_ms": 0.0})
        if obj is None:
            return primitives.result(
                False, f"no '{ctx['source_label']}' detected (saw {len(objs)} objects)")
        if not obj["graspable"]:
            return primitives.result(False, f"'{ctx['source_label']}' is not graspable")
        ctx["object"] = obj
        center = np.asarray(obj["center_base"], float).copy()
        domain = self.cfg["grasp_planner"].get("operating_domain", {})
        xr, yr = domain.get("x"), domain.get("y")
        if ((xr and not xr[0] <= center[0] <= xr[1]) or
                (yr and not yr[0] <= center[1] <= yr[1])):
            ctx["object"] = None
            return primitives.result(False, f"object center {np.round(center, 3)} is outside "
                                     "the tabletop operating domain",
                                     reason="OBJECT_OUT_OF_ODD")
        previous = ctx.get("last_observation_center")
        if previous is not None:
            ctx.setdefault("reobservation_displacements_m", []).append(
                float(np.linalg.norm(center - previous)))
        ctx["last_observation_center"] = center
        ctx.pop("awaiting_reobservation", None)
        ctx["object_z0"] = self.world.body_z(self._body_name(obj["label"]))
        gt_pos = self.world.body_pos(self._body_name(obj["label"]))
        if gt_pos is not None:
            ctx["source_center_error_m"] = float(
                np.linalg.norm(np.asarray(obj["center_base"], float) - gt_pos)
            )
        return primitives.result(
            True, f"selected {obj['id']} conf={obj['confidence']:.2f} "
                  f"base={np.round(obj['center_base'], 3)} ({obj['n_roi']} ROI pts)")

    def _st_plan_pick(self, ctx):
        profile = grasp_planner.resolve_grasp_profile(
            self.cfg, ctx["object"]["label"])
        strategy = profile.strategy
        ctx["grasp_profile"] = profile
        ctx["grasp_strategy"] = strategy
        if strategy in ("multi_candidate_v1", "multi_candidate_v1_1", "multi_candidate_v1.1"):
            body_name = self._body_name(ctx["object"]["label"])
            target_body_id = mujoco.mj_name2id(
                self.world.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            ranked = grasp_planner.plan(
                ctx["object"]["center_base"], self.world, self.cfg,
                target_body_id=(target_body_id if target_body_id >= 0 else None),
                profile=profile)
            ctx["candidate_count"] = len(ranked)
            ctx["candidate_reports"] = [c.public() for c in ranked]
            center = np.asarray(ctx["object"]["center_base"], float)
            def repeated(c):
                return grasp_planner.is_failed_action(
                    center, c, ctx.get("failed_actions", ()), displacement_m=0.010)
            chosen = next((c for c in ranked if c.feasible and not repeated(c)), None)
            if chosen is None:
                return primitives.result(False, "no feasible untried grasp candidate",
                                         reason="NO_FEASIBLE_CANDIDATE")
            ctx["selected_candidate"] = chosen
            ctx["selected_candidate_id"] = chosen.candidate_id
            ctx["selected_direction_deg"] = chosen.direction_deg
            ctx["direction_policy_mode"] = chosen.direction_policy_mode
            ctx["direction_policy_band"] = chosen.direction_policy_band
            ctx["direction_reference_deg"] = chosen.direction_reference_deg
            ctx["planned_grasp_z_m"] = chosen.planned_grasp_z_m
            ctx["grasp_z_clearance_m"] = chosen.grasp_z_clearance_m
            ctx["selected_plan_signature"] = grasp_planner.stable_plan_signature(
                ctx["observation_version"], ctx["object"]["center_base"], chosen)
            ctx["selected_action_fingerprint"] = grasp_planner.action_fingerprint(
                ctx["object"]["center_base"], chosen)
            ctx["pick_plan"] = (chosen.grasp, chosen.grasp_mat, chosen.pre, chosen.lift)
            return primitives.result(
                True, f"selected {chosen.candidate_id} direction={chosen.direction_deg:.0f}deg "
                      f"margin={chosen.joint_margin_rad:.3f}rad")
        ctx["pick_plan"] = primitives.plan_horizontal_grasp(
            ctx["object"]["center_base"], self.cfg)
        grasp, _, pre, _ = ctx["pick_plan"]
        return primitives.result(True, f"grasp={np.round(grasp, 3)} pre={np.round(pre, 3)}")

    def _st_execute_pick(self, ctx):
        candidate = ctx.get("selected_candidate")
        if candidate is not None:
            motion_plan = candidate.motion_plan
            tolerance = float(self.cfg["grasp_planner"].get(
                "plan_start_tolerance_rad", 0.03))
            error = float(np.max(np.abs(
                np.asarray(self.world.data.qpos[:6]) - motion_plan.start_q)))
            if error > tolerance:
                return primitives.result(
                    False, f"current joints differ from planned start by {error:.4f} rad",
                    reason="PLAN_START_STATE_MISMATCH", start_error_rad=error)
            motion_cfg = self.cfg.get("motion", {})
            execute = (primitives.execute_continuous_motion_plan
                       if motion_cfg.get("execution_mode") == "continuous"
                       else primitives.execute_motion_plan)
            kwargs = {
                "target_body_name": self._body_name(ctx["object"]["label"]),
                "lift_threshold_m": float(
                    self.cfg["grasp"]["lift_success_threshold"]),
                "candidate_id": candidate.candidate_id,
                "direction_deg": candidate.direction_deg,
            }
            if execute is primitives.execute_continuous_motion_plan:
                kwargs["config"] = primitives.TrajectoryExecutionConfig.from_mapping(
                    motion_cfg)
            execution = execute(self.world, motion_plan, **kwargs)
            ctx["execution_trace"] = execution.get("data", {}).get("execution_trace")
            ctx["trajectory_reports"] = execution.get("data", {}).get(
                "trajectory_reports")
            ctx["simulated_motion_s"] = execution.get("data", {}).get(
                "simulated_motion_s")
            return execution
        # Legacy radial_single retains its established execution behavior.
        if self.world.use_analytic_ik and self.world.ready_ctrl is not None:
            self.world.move_to_pose(self.world.ready_ctrl, steps=900)
        return primitives.execute_grasp(self.world, *ctx["pick_plan"])

    def _st_verify_grasp(self, ctx):
        rose = self.world.body_z(self._body_name(ctx["object"]["label"])) - ctx["object_z0"]
        ctx["rose_m"] = rose
        thresh = self.cfg["grasp"]["lift_success_threshold"]
        if rose > thresh:
            return primitives.result(True, f"object lifted {rose * 1000:.0f} mm")
        if ctx.get("selected_candidate_id"):
            ctx.setdefault("failed_candidate_ids", []).append(ctx["selected_candidate_id"])
        if ctx.get("selected_plan_signature"):
            ctx.setdefault("failed_plan_signatures", []).append(ctx["selected_plan_signature"])
        if ctx.get("selected_action_fingerprint"):
            ctx.setdefault("failed_actions", []).append({
                "center": np.asarray(ctx["object"]["center_base"], float).copy(),
                "fingerprint": ctx["selected_action_fingerprint"]})
        return primitives.result(
            False, f"object only rose {rose * 1000:.0f} mm (< {thresh * 1000:.0f} mm)",
            reason="LIFT_THRESHOLD_NOT_MET")

    def _st_recover_pick(self, ctx):
        """Open, move to the validated view pose, and invalidate stale perception."""
        primitives.open_gripper(self.world)
        pose = self.cfg["grasp_planner"].get("safe_observation_pose")
        if pose is None:
            return primitives.result(False, "safe observation pose is not configured",
                                     reason="SAFE_OBSERVATION_POSE_MISSING")
        self.world.move_to_pose(np.asarray(pose, float), steps=900)
        stale = ctx.pop("object", None)
        ctx.pop("registry", None)
        ctx.pop("pick_plan", None)
        ctx.pop("selected_candidate", None)
        ctx.pop("selected_candidate_id", None)
        ctx.pop("selected_plan_signature", None)
        ctx.pop("selected_action_fingerprint", None)
        ctx["invalidated_observation_version"] = ctx.get("observation_version")
        ctx["awaiting_reobservation"] = True
        ctx["stale_object_id"] = stale.get("id") if stale else None
        return primitives.result(True, "gripper opened; safe observation pose reached; "
                                 "old ObjectObservation invalidated")

    # ---- place states ------------------------------------------------------
    def _st_detect_target(self, ctx):
        # Resolve the container from the registry detected at observation time.
        objs = ctx.get("registry", [])
        if ctx.get("container_label"):
            target = object_registry.get_object(objs, ctx["container_label"], "nearest")
        else:
            containers = [o for o in objs if o["container"]]
            target = min(containers, key=lambda o: float(np.linalg.norm(o["center_base"][:2]))) \
                if containers else None
        if target is None:
            return primitives.result(
                False, "no container detected (sim domain gap: see DEVLOG; pass "
                       "container_xyz to place at a known container location)")
        ctx["place_xyz"] = np.asarray(target["center_base"], float)
        ctx["container"] = target
        gt_pos = self.world.body_pos(self._body_name(target["label"]))
        if gt_pos is not None:
            gt_pos = np.asarray(gt_pos, float).copy()
            gt_pos[2] = self.cfg["scene"]["table_top_z"] + object_registry.CONTAINER_PLANE_OFFSET
            ctx["target_center_error_m"] = float(
                np.linalg.norm(np.asarray(target["center_base"], float) - gt_pos)
            )
        return primitives.result(True, f"target container {target['id']} "
                                       f"base={np.round(target['center_base'], 3)}")

    def _st_plan_place(self, ctx):
        key = ("container_release_z_offset" if ctx["place_kind"] == "container"
               else "surface_release_z_offset")
        ctx["release_z"] = self.cfg["place"][key]
        ctx["place_point"] = ctx["place_xyz"]
        return primitives.result(
            True, f"release above {np.round(ctx['place_point'], 3)} +{ctx['release_z']:.2f}m")

    def _st_execute_place(self, ctx):
        return primitives.release_at(self.world, ctx["place_point"], ctx["release_z"])

    def _st_verify_done(self, ctx):
        pos = self.world.body_pos(self._body_name(ctx["object"]["label"]))
        dxy = float(np.linalg.norm(pos[:2] - ctx["place_point"][:2]))
        ctx["place_dxy_m"] = dxy
        tol = self.cfg["place"]["xy_tolerance"]
        if dxy <= tol:
            return primitives.result(True, f"object placed {dxy * 1000:.0f} mm from target")
        return primitives.result(
            False, f"object landed {dxy * 1000:.0f} mm from target (> {tol * 1000:.0f} mm)")

    # ---- summary -----------------------------------------------------------
    def _summary(self, task, target, success, ctx):
        obj = ctx.get("object")
        data = {
            "n_detected": ctx.get("n_detected"),
            "selected": obj["id"] if obj else None,
            "center_base": obj["center_base"] if obj else None,
            "rose_mm": round(ctx["rose_m"] * 1000, 1) if "rose_m" in ctx else None,
            "place_point": ctx.get("place_point"),
            "place_dxy_mm": round(ctx["place_dxy_m"] * 1000, 1) if "place_dxy_m" in ctx else None,
            "source_center_error_mm": round(ctx["source_center_error_m"] * 1000, 1)
            if "source_center_error_m" in ctx else None,
            "target_center_error_mm": round(ctx["target_center_error_m"] * 1000, 1)
            if "target_center_error_m" in ctx else None,
            "task_duration_ms": round(
                (time.perf_counter() - ctx["_task_started_at"]) * 1000.0, 1
            ) if "_task_started_at" in ctx else None,
            "grasp_strategy": ctx.get("grasp_strategy", self.cfg["grasp"].get("strategy", "radial_single")),
            "candidate_count": ctx.get("candidate_count", 1),
            "selected_candidate_id": ctx.get(
                "selected_candidate_id",
                "radial_single" if ctx.get("grasp_strategy") == "radial_single" else None),
            "selected_direction_deg": ctx.get("selected_direction_deg"),
            "direction_policy_mode": ctx.get("direction_policy_mode"),
            "direction_policy_band": ctx.get("direction_policy_band"),
            "direction_reference_deg": ctx.get("direction_reference_deg"),
            "planned_grasp_z_m": ctx.get("planned_grasp_z_m"),
            "grasp_z_clearance_m": ctx.get("grasp_z_clearance_m"),
            "selected_joint_margin_rad": getattr(ctx.get("selected_candidate"), "joint_margin_rad", None),
            "max_cartesian_deviation_mm": getattr(
                ctx.get("selected_candidate"), "max_cartesian_deviation_mm", None),
            "lift_horizontal_path_drift_mm": getattr(
                ctx.get("selected_candidate"), "lift_horizontal_path_drift_mm", None),
            "grasp_profile": ({
                "strategy": ctx["grasp_profile"].strategy,
                "preferred_directions_deg": ctx["grasp_profile"].preferred_directions_deg,
                "direction_policy_mode": ctx["grasp_profile"].direction_policy_mode,
                "direction_split_y_m": ctx["grasp_profile"].direction_split_y_m,
                "fallback_directions_deg": ctx["grasp_profile"].fallback_directions_deg,
                "grasp_z_clearance_m": ctx["grasp_profile"].grasp_z_clearance_m,
                "final_approach_mode": ctx["grasp_profile"].final_approach_mode,
                "lift_mode": ctx["grasp_profile"].lift_mode,
                "cartesian_waypoint_count": ctx["grasp_profile"].cartesian_waypoint_count,
            } if ctx.get("grasp_profile") else None),
            "execution_trace": ctx.get("execution_trace"),
            "trajectory_reports": ctx.get("trajectory_reports"),
            "simulated_motion_s": ctx.get("simulated_motion_s"),
            "failed_candidate_ids": ctx.get("failed_candidate_ids", []),
            "failed_plan_signatures": ctx.get("failed_plan_signatures", []),
            "failed_actions": ctx.get("failed_actions", []),
            "observation_version": ctx.get("observation_version"),
            "invalidated_observation_version": ctx.get("invalidated_observation_version"),
            "reobservation_displacements_mm": [round(v * 1000.0, 2) for v in
                                                ctx.get("reobservation_displacements_m", [])],
            "candidate_reports": ctx.get("candidate_reports", []),
        }
        return _native({"task": task, "target": target, "success": success,
                        "states": self.log, "data": data})


def _save_evidence(world, name, max_frames=120, scale=0.5):
    from PIL import Image
    EVIDENCE_DIR.mkdir(parents=True, exist_ok=True)
    if not world.frames:
        return
    Image.fromarray(world.frames[-1]).save(EVIDENCE_DIR / f"{name}_final.png")
    # cap frame count + downscale so long tasks don't produce huge gifs
    step = max(1, len(world.frames) // max_frames)
    sel = world.frames[::step]
    gif = []
    for arr in sel:
        im = Image.fromarray(arr)
        if scale != 1.0:
            im = im.resize((int(im.width * scale), int(im.height * scale)))
        gif.append(im)
    gif[0].save(EVIDENCE_DIR / f"{name}.gif", save_all=True, append_images=gif[1:],
                duration=80, loop=0, optimize=True)
    print(f"saved {EVIDENCE_DIR / (name + '_final.png')} and {name}.gif ({len(gif)} frames)")


def _parse_xyz(s):
    return [float(v) for v in s.split(",")]


def main():
    parser = argparse.ArgumentParser(description="Run a multitask FSM task.")
    parser.add_argument("--task", default="pick",
                        choices=["pick", "place_at", "place_into", "clear_table"])
    parser.add_argument("--target", default="cup", help="source object label")
    parser.add_argument("--policy", default="nearest")
    parser.add_argument("--grasp-strategy",
                        choices=["radial_single", "multi_candidate_v1", "multi_candidate_v1_1"])
    parser.add_argument("--xyz", type=_parse_xyz, default=None,
                        help="place_at target / place_into container coord, 'x,y,z'")
    parser.add_argument("--container", default=None, help="place_into container label (vision)")
    parser.add_argument("--perception", default=None, choices=["yolo", "sim_gt"],
                        help="override detector.backend; sim_gt uses MuJoCo body poses")
    parser.add_argument("--model", default=None,
                        choices=["menagerie", "piper_real", "real"],
                        help="override model.active; 'real' = DH-matching piper_real + analytic IK")
    parser.add_argument("--labels", default=None, help="clear_table source labels, comma-separated")
    parser.add_argument("--record-cam", default="world_cam", choices=["world_cam", "wrist_cam"],
                        help="camera for the evidence gif/png")
    parser.add_argument("--annotate", action="store_true",
                        help="draw YOLO detection boxes on the recorded frames")
    parser.add_argument("--no-evidence", action="store_true")
    args = parser.parse_args()

    cfg = load_config()
    if args.perception is not None:
        cfg["detector"]["backend"] = args.perception
    if args.grasp_strategy is not None:
        cfg["grasp"]["strategy"] = args.grasp_strategy
    # Resolve the robot model / IK backend (menagerie numeric IK vs piper_real analytic IK).
    mcfg = cfg.get("model", {})
    active = args.model or mcfg.get("active", "menagerie")
    if active == "real":
        warnings.warn("--model real is deprecated; use --model piper_real",
                      DeprecationWarning, stacklevel=2)
    canonical_active = "piper_real" if active == "real" else active
    config_active = "real" if canonical_active == "piper_real" else canonical_active
    msel = mcfg.get(config_active, {})
    scene_path = (ROOT / msel["scene"]) if msel.get("scene") else None
    analytic_ik = bool(msel.get("analytic_ik", False))
    if msel.get("scan_poses"):
        cfg["detector"]["scan_poses"] = msel["scan_poses"]
    for section in ("grasp", "place", "scene", "detector"):   # per-model parameter overrides
        if isinstance(msel.get(section), dict):
            cfg[section].update(msel[section])
    print(f"[model] {canonical_active}  scene={scene_path}  analytic_ik={analytic_ik}  "
          f"perception={cfg['detector']['backend']}")
    labels = list(cfg["objects"]["graspable"]) + list(cfg["objects"]["containers"])
    world = SimWorld.load(scene=scene_path, record=not args.no_evidence,
                          record_cam=args.record_cam,
                          annotate_labels=(labels if args.annotate else ()),
                          annotate_conf=cfg["detector"]["confidence_threshold"],
                          use_analytic_ik=analytic_ik)
    ex = TaskExecutor(world, cfg)
    if args.task == "pick":
        summary = ex.pick(args.target, args.policy)
    elif args.task == "place_at":
        if args.xyz is None:
            parser.error("place_at requires --xyz x,y,z")
        summary = ex.place_at(args.target, args.xyz, args.policy)
    elif args.task == "place_into":
        summary = ex.place_into(args.target, args.container, args.xyz, args.policy)
    else:  # clear_table
        labels = (args.labels or args.target).split(",")
        summary = ex.clear_table(labels, args.container, args.xyz, args.policy)

    print("\n=== task result (JSON) ===")
    print(json.dumps(summary, indent=2))
    if not args.no_evidence:
        name = f"{args.task}_{args.target}"
        if canonical_active != "menagerie":
            name += f"_{canonical_active}"
        if args.record_cam != "world_cam":
            name += f"_{args.record_cam}"
        if args.annotate:
            name += "_yolo"
        _save_evidence(world, name)
    raise SystemExit(0 if summary["success"] else 1)


if __name__ == "__main__":
    main()
