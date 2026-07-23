"""Perception-only bounded-orientation grasp planning for Piper."""
from dataclasses import dataclass, field
from enum import Enum

import mujoco
import numpy as np

from .primitives import grasp_sim
from .motion_plan import (CollisionTrace, JointWaypoint, MotionPhase, MotionPlan,
                          MotionSegment)
from sim.constrained_ik import (IKConstraints, IKRequest, IKStage,
                                solve_constrained_ik)


class ConstraintLevel(str, Enum):
    EXACT = "EXACT"
    RELAXED_ORIENTATION = "RELAXED_ORIENTATION"


class ContactCategory(str, Enum):
    STATIC_ENVIRONMENT = "STATIC_ENVIRONMENT"
    ADJACENT_SELF = "ADJACENT_SELF"
    NON_ADJACENT_SELF = "NON_ADJACENT_SELF"
    TARGET_OBJECT = "TARGET_OBJECT"
    UNMODELED_MOVABLE = "UNMODELED_MOVABLE"


@dataclass(frozen=True)
class GraspProfile:
    strategy: str
    preferred_directions_deg: tuple = ()
    direction_policy_mode: str = "fixed"
    direction_split_y_m: float = -0.08
    lower_direction_reference: str = "radial"
    lower_direction_offset_deg: float = -15.0
    upper_direction_reference: str = "absolute"
    upper_direction_angle_deg: float = 0.0
    fallback_directions_deg: tuple = ()
    grasp_z_clearance_m: float | None = None
    final_approach_mode: str = "joint"
    lift_mode: str = "joint"
    cartesian_waypoint_count: int = 3


def resolve_grasp_profile(cfg, target_label):
    raw = cfg.get("grasp", {}).get("profiles", {}).get(target_label, {})
    policy = raw.get("direction_policy", {})
    lower = policy.get("lower_band", {})
    upper = policy.get("upper_band", {})
    return GraspProfile(
        strategy=raw.get("strategy", cfg.get("grasp", {}).get(
            "strategy", "radial_single")),
        preferred_directions_deg=tuple(float(v) for v in
                                       raw.get("preferred_directions_deg", ())),
        direction_policy_mode=policy.get("mode", "fixed"),
        direction_split_y_m=float(policy.get("split_y_m", -0.08)),
        lower_direction_reference=lower.get("reference", "radial"),
        lower_direction_offset_deg=float(lower.get("offset_deg", -15.0)),
        upper_direction_reference=upper.get("reference", "absolute"),
        upper_direction_angle_deg=float(upper.get("angle_deg", 0.0)),
        fallback_directions_deg=tuple(float(v) for v in
                                      raw.get("fallback_directions_deg", ())),
        grasp_z_clearance_m=(float(raw["grasp_z_clearance_m"])
                             if raw.get("grasp_z_clearance_m") is not None else None),
        final_approach_mode=raw.get("final_approach_mode", "joint"),
        lift_mode=raw.get("lift_mode", "joint"),
        cartesian_waypoint_count=int(raw.get(
            "cartesian_waypoint_count",
            cfg.get("grasp_planner", {}).get("cartesian_waypoint_count", 3))),
    )


@dataclass(frozen=True)
class CollisionResult:
    allowed: bool
    phase: str
    collision_scope: str = "static_and_self"
    contact_category: str = "NONE"
    reason: str = "NO_HARD_COLLISION"


def contact_allowed(phase, category):
    """Collision policy contract, independent of backend contact classification."""
    phase, category = MotionPhase(phase), ContactCategory(category)
    if category == ContactCategory.ADJACENT_SELF:
        return True
    if category == ContactCategory.TARGET_OBJECT:
        return phase in (MotionPhase.FINAL_APPROACH, MotionPhase.CLOSING)
    return False


@dataclass(frozen=True)
class GraspConstraint:
    tcp_position: np.ndarray
    tcp_tolerance_m: float = 0.002
    approach_direction: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    max_approach_error_deg: float = 20.0
    max_closing_axis_vertical_deg: float = 10.0
    wrist_equivalent_deg: tuple = (0.0, 180.0)


@dataclass
class GraspCandidate:
    candidate_id: str
    direction_index: int
    direction_deg: float
    grasp: np.ndarray
    grasp_mat: np.ndarray
    pre: np.ndarray
    lift: np.ndarray
    preference_rank: int = 0
    direction_policy_mode: str = "fixed"
    direction_policy_band: str | None = None
    direction_reference_deg: float | None = None
    planned_grasp_z_m: float | None = None
    grasp_z_clearance_m: float | None = None
    feasible: bool = False
    rejection_reasons: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    joint_margin_rad: float = 0.0
    tcp_error_m: float = float("inf")
    path_length_m: float = 0.0
    joint_delta_rad: float = float("inf")
    collision_samples: int = 0
    constraint_level: str = ConstraintLevel.EXACT.value
    approach_error_deg: float = float("inf")
    closing_axis_vertical_deg: float = float("inf")
    relaxation_penalty: float = float("inf")
    collision_scope: str = "static_and_self"
    solutions: dict = field(default_factory=dict, repr=False)
    ik_results: dict = field(default_factory=dict, repr=False)
    motion_plan: object = field(default=None, repr=False)
    direct_collision_trace: list = field(default_factory=list, repr=False)
    detour_collision_trace: list = field(default_factory=list, repr=False)
    max_cartesian_deviation_mm: float = 0.0
    lift_horizontal_path_drift_mm: float = 0.0

    def public(self):
        return {
            "candidate_id": self.candidate_id,
            "direction_index": self.direction_index,
            "direction_deg": round(self.direction_deg, 1),
            "preference_rank": self.preference_rank,
            "direction_policy_mode": self.direction_policy_mode,
            "direction_policy_band": self.direction_policy_band,
            "direction_reference_deg": (round(float(self.direction_reference_deg), 2)
                                        if self.direction_reference_deg is not None else None),
            "planned_grasp_z_m": (round(float(self.planned_grasp_z_m), 4)
                                  if self.planned_grasp_z_m is not None else None),
            "grasp_z_clearance_m": (round(float(self.grasp_z_clearance_m), 4)
                                    if self.grasp_z_clearance_m is not None else None),
            "feasible": self.feasible,
            "rejection_reasons": list(self.rejection_reasons),
            "warnings": list(self.warnings),
            "joint_margin_rad": round(float(self.joint_margin_rad), 4),
            "tcp_error_mm": round(float(self.tcp_error_m) * 1000.0, 2),
            "path_length_m": round(float(self.path_length_m), 4),
            "joint_delta_rad": round(float(self.joint_delta_rad), 4),
            "collision_samples": int(self.collision_samples),
            "collision_free": self.collision_samples == 0,
            "collision_scope": self.collision_scope,
            "constraint_level": self.constraint_level,
            "approach_error_deg": round(float(self.approach_error_deg), 2),
            "closing_axis_vertical_deg": round(float(self.closing_axis_vertical_deg), 2),
            "relaxation_penalty": round(float(self.relaxation_penalty), 4),
            "max_cartesian_deviation_mm": round(
                float(self.max_cartesian_deviation_mm), 3),
            "lift_horizontal_path_drift_mm": round(
                float(self.lift_horizontal_path_drift_mm), 3),
            "ik_results": {name: {
                "solve_path": r.solve_path, "iterations": r.iterations,
                "elapsed_ms": round(r.elapsed_ms, 2),
                "termination_reason": r.termination_reason, "cache_hit": r.cache_hit,
            } for name, r in self.ik_results.items()},
            "motion_plan": self.motion_plan.public() if self.motion_plan else None,
            "direct_collision_trace": [t.public() for t in self.direct_collision_trace],
            "detour_collision_trace": [t.public() for t in self.detour_collision_trace],
        }


def _frame(direction):
    direction = np.asarray(direction, float)
    direction /= np.linalg.norm(direction)
    closing = np.cross(direction, [0.0, 0.0, 1.0])
    return np.column_stack(([0.0, 0.0, 1.0], closing / np.linalg.norm(closing), direction))


def select_preferred_directions(center_base, profile):
    """Return perception-only preferred approach directions for a grasp profile."""
    if profile.direction_policy_mode == "fixed":
        return tuple(float(v) % 360.0 for v in profile.preferred_directions_deg)
    if profile.direction_policy_mode != "lateral_band":
        raise ValueError(f"unsupported direction policy: {profile.direction_policy_mode}")
    center = np.asarray(center_base, float)
    if center[1] <= profile.direction_split_y_m:
        if profile.lower_direction_reference != "radial":
            raise ValueError("lower lateral band currently requires radial reference")
        reference = float(np.rad2deg(np.arctan2(center[1], center[0])))
        primary = reference + profile.lower_direction_offset_deg
    else:
        if profile.upper_direction_reference != "absolute":
            raise ValueError("upper lateral band currently requires absolute reference")
        primary = profile.upper_direction_angle_deg
    return tuple(float(v) % 360.0 for v in
                 (primary, *profile.fallback_directions_deg))


def _direction_policy_context(center, profile):
    if profile is None or profile.direction_policy_mode == "fixed":
        return "fixed", None, None
    lower = center[1] <= profile.direction_split_y_m
    if lower:
        reference = float(np.rad2deg(np.arctan2(center[1], center[0]))) % 360.0
        return profile.direction_policy_mode, "lower", reference
    return (profile.direction_policy_mode, "upper",
            profile.upper_direction_angle_deg % 360.0)


def resolve_grasp_z(observed_center, table_z, global_clearance_m, profile=None,
                    use_table_grasp_z=False):
    """Resolve grasp height while keeping object-specific overrides isolated."""
    if profile is not None and profile.grasp_z_clearance_m is not None:
        return float(table_z) + float(profile.grasp_z_clearance_m)
    floor = float(table_z) + float(global_clearance_m)
    return floor if use_table_grasp_z else max(float(observed_center[2]), floor)


def generate_candidates(center_base, cfg, profile=None):
    center = np.asarray(center_base, float).copy()
    center[2] = resolve_grasp_z(
        center, cfg["scene"]["table_top_z"],
        cfg["grasp"]["min_grasp_z_clearance"], profile,
        use_table_grasp_z=bool(cfg["grasp"].get("use_table_grasp_z")))
    count = int(cfg["grasp_planner"].get("candidate_count", 8))
    start = (np.rad2deg(np.arctan2(center[1], center[0])) +
             float(cfg["grasp_planner"].get("start_angle_offset_deg", 0.0)))
    preferred = select_preferred_directions(center, profile) if profile else ()
    policy_mode, policy_band, primary_reference = _direction_policy_context(
        center, profile)
    if not preferred:
        angles = [(start + i * 360.0 / count) % 360.0 for i in range(count)]
        preferred_count = 0
    else:
        angles = []
        for value in preferred:
            deg = float(value) % 360.0
            if not any(abs(((deg - old + 180) % 360) - 180) < 1e-6 for old in angles):
                angles.append(deg)
        preferred_count = len(angles)
        for i in range(count * 2):
            deg = (start + i * 360.0 / count) % 360.0
            if not any(abs(((deg - old + 180) % 360) - 180) < 1e-6 for old in angles):
                angles.append(deg)
            if len(angles) >= count:
                break
        angles = angles[:count]
    out = []
    for i, deg in enumerate(angles):
        direction = np.array([np.cos(np.deg2rad(deg)), np.sin(np.deg2rad(deg)), 0.0])
        mat = _frame(direction)
        distance = float(cfg["grasp"]["approach_distance"])
        lift_h = float(cfg["grasp"]["lift_height"])
        prefix = "preferred" if i < preferred_count else "radial"
        reference = (primary_reference if i == 0 and primary_reference is not None
                     else deg)
        out.append(GraspCandidate(
            f"{prefix}_{i:02d}", i, deg, center.copy(), mat,
            center - distance * direction, center + [0.0, 0.0, lift_h],
            preference_rank=(i if preferred_count else 0),
            direction_policy_mode=policy_mode,
            direction_policy_band=policy_band,
            direction_reference_deg=reference,
            planned_grasp_z_m=float(center[2]),
            grasp_z_clearance_m=float(center[2] - cfg["scene"]["table_top_z"]),
            path_length_m=distance + lift_h))
    return out


def action_fingerprint(center, candidate):
    center_bucket = tuple(int(round(float(v) / 0.010)) for v in np.asarray(center)[:3])
    direction_bucket = int(round((float(candidate.direction_deg) % 360.0) / 5.0)) % 72
    return center_bucket, direction_bucket, candidate.constraint_level


def stable_plan_signature(observation_version, center, candidate):
    """Log metadata. Retry exclusion uses action_fingerprint, without version."""
    return int(observation_version), action_fingerprint(center, candidate)


def is_failed_action(center, candidate, failed_actions, displacement_m=0.010):
    fingerprint = action_fingerprint(center, candidate)
    return any(np.linalg.norm(np.asarray(center, float) - np.asarray(f["center"], float))
               <= displacement_m and fingerprint[1:] == tuple(f["fingerprint"][1:])
               for f in failed_actions)


def _free_body(model, geom_id):
    body = int(model.geom_bodyid[geom_id])
    while body > 0:
        jadr = int(model.body_jntadr[body])
        if model.body_jntnum[body] and model.jnt_type[jadr] == mujoco.mjtJoint.mjJNT_FREE:
            return True
        body = int(model.body_parentid[body])
    return False


def _robot_body(model, geom_id):
    body = int(model.geom_bodyid[geom_id])
    while body > 0:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
        if any(t in name.lower() for t in ("link", "gripper", "finger", "piper")):
            return True
        body = int(model.body_parentid[body])
    return False


def _name(model, obj_type, obj_id):
    return mujoco.mj_id2name(model, obj_type, int(obj_id)) or f"id:{int(obj_id)}"


def collision_traces_at(model, base_data, q, phase, segment_name,
                        sample_index, alpha, target_body_id=None):
    """Return every robot-related contact with actionable provenance."""
    data = mujoco.MjData(model); data.qpos[:] = base_data.qpos; data.qpos[:6] = q
    mujoco.mj_forward(model, data)
    traces = []
    for con in data.contact:
        r1, r2 = _robot_body(model, con.geom1), _robot_body(model, con.geom2)
        if not (r1 or r2):
            continue
        b1, b2 = int(model.geom_bodyid[con.geom1]), int(model.geom_bodyid[con.geom2])
        adjacent = int(model.body_parentid[b1]) == b2 or int(model.body_parentid[b2]) == b1
        if r1 and r2:
            category = (ContactCategory.ADJACENT_SELF if adjacent
                        else ContactCategory.NON_ADJACENT_SELF)
        else:
            other_body = b2 if r1 else b1
            if target_body_id is not None and other_body == target_body_id:
                category = ContactCategory.TARGET_OBJECT
            elif _free_body(model, con.geom2 if r1 else con.geom1):
                category = ContactCategory.UNMODELED_MOVABLE
            else:
                category = ContactCategory.STATIC_ENVIRONMENT
        allowed = (category == ContactCategory.UNMODELED_MOVABLE or
                   contact_allowed(phase, category))
        reason = "CONTACT_ALLOWED" if allowed else f"HARD_COLLISION_{category.value}"
        traces.append(CollisionTrace(
            MotionPhase(phase).value, segment_name, sample_index, float(alpha),
            np.asarray(q, float).copy(),
            (_name(model, mujoco.mjtObj.mjOBJ_GEOM, con.geom1),
             _name(model, mujoco.mjtObj.mjOBJ_GEOM, con.geom2)),
            (_name(model, mujoco.mjtObj.mjOBJ_BODY, b1),
             _name(model, mujoco.mjtObj.mjOBJ_BODY, b2)),
            category.value, float(con.dist), allowed, reason))
    return traces


def check_collision(model, base_data, q, phase=MotionPhase.TRANSIT,
                    target_body_id=None):
    traces = collision_traces_at(model, base_data, q, phase, "point", 0, 0.0,
                                 target_body_id)
    hard = next((t for t in traces if not t.allowed), None)
    return (CollisionResult(False, MotionPhase(phase).value,
                            contact_category=hard.contact_category, reason=hard.reason)
            if hard else CollisionResult(True, MotionPhase(phase).value))


def validate_motion_segment(model, base_data, segment, max_joint_step_rad,
                            target_body_id=None):
    traces, sample_index = [], 0
    for a, b in zip(segment.waypoints, segment.waypoints[1:]):
        max_delta = float(np.max(np.abs(np.asarray(b.q) - np.asarray(a.q))))
        subdivisions = max(1, int(np.ceil(max_delta / max_joint_step_rad)))
        for local_index, alpha in enumerate(np.linspace(0.0, 1.0, subdivisions + 1)):
            if sample_index and local_index == 0:
                continue
            q = np.asarray(a.q) + alpha * (np.asarray(b.q) - np.asarray(a.q))
            traces.extend(collision_traces_at(
                model, base_data, q, segment.phase, segment.name,
                sample_index, alpha, target_body_id))
            sample_index += 1
    segment.collision_trace = traces
    return not any(not t.allowed for t in traces)


def _hard_limits(world):
    lo = np.asarray(world.model.jnt_range[:6, 0], float).copy()
    hi = np.asarray(world.model.jnt_range[:6, 1], float).copy()
    if world.use_analytic_ik:
        arm = grasp_sim._piper_arm()
        lo = np.maximum(lo, np.deg2rad([v[0] for v in arm.link_limits]))
        hi = np.minimum(hi, np.deg2rad([v[1] for v in arm.link_limits]))
    return lo, hi


def _joint_margin(lo, hi, q):
    return float(np.min(np.minimum(np.asarray(q) - lo, hi - np.asarray(q))))


def _solve_stage(world, candidate, stage, pos, seed, cfg):
    pcfg = cfg["grasp_planner"]
    lo, hi = _hard_limits(world)
    exact = None
    if world.use_analytic_ik:
        exact = lambda req, qseed: grasp_sim.ik_target_analytic_exact(
            req.data, req.site_id, req.target_position, candidate.grasp_mat, qseed)
    request = IKRequest(
        world.model, world.data, world.tcp, np.asarray(pos, float),
        candidate.grasp_mat[:, 2], candidate.grasp_mat[:, 1],
        IKConstraints(float(pcfg.get("tcp_tolerance_m", .002)),
                      float(pcfg.get("max_approach_error_deg", 20.)),
                      float(pcfg.get("max_closing_axis_vertical_deg", 10.))),
        (np.asarray(seed), np.asarray(world.data.qpos[:6]), (lo + hi) / 2.0), stage,
        int(pcfg.get("ik_max_iterations", 180)),
        float(pcfg.get("ik_time_budget_s", .20)), exact_solver=exact,
        hard_lower=lo, hard_upper=hi)
    return solve_constrained_ik(request)


def _direct_motion_plan(start_q, candidate):
    return MotionPlan(np.asarray(start_q).copy(), [
        MotionSegment("start_to_pre", MotionPhase.TRANSIT.value,
                      [JointWaypoint("start", np.asarray(start_q)),
                       JointWaypoint("pre", candidate.solutions["pre"])]),
        MotionSegment("pre_to_grasp", MotionPhase.FINAL_APPROACH.value,
                      [JointWaypoint("pre", candidate.solutions["pre"]),
                       JointWaypoint("grasp", candidate.solutions["grasp"])]),
        MotionSegment("grasp_to_lift", MotionPhase.LIFT.value,
                      [JointWaypoint("grasp", candidate.solutions["grasp"]),
                       JointWaypoint("lift", candidate.solutions["lift"])]),
    ])


def build_cartesian_segment(world, candidate, name, phase, start_pos, end_pos,
                            start_q, end_q, waypoint_count, cfg, ik_stage=None):
    """Build a bounded task-space line using the existing constrained IK path."""
    phase = MotionPhase(phase)
    stage = ik_stage or (IKStage.LIFT if phase == MotionPhase.LIFT else IKStage.GRASP)
    waypoints = [JointWaypoint(name + "_start", np.asarray(start_q))]
    seed = np.asarray(start_q)
    for i in range(1, int(waypoint_count) + 1):
        alpha = i / (int(waypoint_count) + 1)
        pos = np.asarray(start_pos) + alpha * (np.asarray(end_pos) - np.asarray(start_pos))
        result = _solve_stage(world, candidate, stage, pos, seed, cfg)
        if not result.success:
            return None
        seed = result.q
        waypoints.append(JointWaypoint(f"{name}_cart_{i:02d}", np.asarray(result.q)))
    waypoints.append(JointWaypoint(name + "_end", np.asarray(end_q)))
    return MotionSegment(name, phase.value, waypoints)


def _profile_motion_plan(world, candidate, start_q, cfg, profile):
    segments = [MotionSegment(
        "start_to_pre", MotionPhase.TRANSIT.value,
        [JointWaypoint("start", np.asarray(start_q)),
         JointWaypoint("pre", candidate.solutions["pre"])])]
    if profile.final_approach_mode == "cartesian":
        approach = build_cartesian_segment(
            world, candidate, "pre_to_grasp", MotionPhase.FINAL_APPROACH,
            candidate.pre, candidate.grasp, candidate.solutions["pre"],
            candidate.solutions["grasp"], profile.cartesian_waypoint_count, cfg)
    else:
        approach = MotionSegment(
            "pre_to_grasp", MotionPhase.FINAL_APPROACH.value,
            [JointWaypoint("pre", candidate.solutions["pre"]),
             JointWaypoint("grasp", candidate.solutions["grasp"])])
    if approach is None:
        return None
    segments.append(approach)
    if profile.lift_mode == "cartesian":
        lift = build_cartesian_segment(
            world, candidate, "grasp_to_lift", MotionPhase.LIFT,
            candidate.grasp, candidate.lift, candidate.solutions["grasp"],
            candidate.solutions["lift"], profile.cartesian_waypoint_count, cfg)
    else:
        lift = MotionSegment(
            "grasp_to_lift", MotionPhase.LIFT.value,
            [JointWaypoint("grasp", candidate.solutions["grasp"]),
             JointWaypoint("lift", candidate.solutions["lift"])])
    if lift is None:
        return None
    segments.append(lift)
    return MotionPlan(np.asarray(start_q).copy(), segments)


def _segment_tcp_positions(world, segment):
    data = mujoco.MjData(world.model)
    data.qpos[:] = world.data.qpos
    positions = []
    for waypoint in segment.waypoints:
        data.qpos[:6] = waypoint.q
        mujoco.mj_forward(world.model, data)
        positions.append(np.asarray(data.site_xpos[world.tcp], float).copy())
    return positions


def _path_deviation_metrics(world, candidate, motion_plan):
    approach = next(s for s in motion_plan.segments if s.name == "pre_to_grasp")
    lift = next(s for s in motion_plan.segments if s.name == "grasp_to_lift")
    approach_positions = _segment_tcp_positions(world, approach)
    start, end = np.asarray(candidate.pre), np.asarray(candidate.grasp)
    vector = end - start
    length2 = float(np.dot(vector, vector))
    deviations = []
    for point in approach_positions:
        alpha = float(np.clip(np.dot(point - start, vector) / length2, 0.0, 1.0))
        deviations.append(float(np.linalg.norm(point - (start + alpha * vector))))
    lift_positions = _segment_tcp_positions(world, lift)
    horizontal = [float(np.linalg.norm(point[:2] - candidate.grasp[:2]))
                  for point in lift_positions]
    return 1000.0 * max(deviations, default=0.0), 1000.0 * max(horizontal, default=0.0)


def _validate_motion_plan(world, motion_plan, cfg, target_body_id=None):
    max_step = float(cfg["grasp_planner"].get("max_joint_step_rad", 0.05))
    return all(validate_motion_segment(world.model, world.data, segment, max_step,
                                       target_body_id)
               for segment in motion_plan.segments)


def _task_space_segment(world, candidate, name, phase, start_pos, end_pos,
                        start_q, end_q, stage, cfg):
    return build_cartesian_segment(
        world, candidate, name, phase, start_pos, end_pos, start_q, end_q,
        int(cfg["grasp_planner"].get("cartesian_waypoint_count", 3)), cfg, stage)


def _detour_motion_plan(world, candidate, direct, cfg, target_body_id=None):
    """Bounded deterministic local repair; only invoked for colliding direct paths."""
    repaired = []
    max_step = float(cfg["grasp_planner"].get("max_joint_step_rad", 0.05))
    for segment in direct.segments:
        hard = any(not t.allowed for t in segment.collision_trace)
        if not hard:
            repaired.append(MotionSegment(segment.name, segment.phase,
                                          list(segment.waypoints)))
            continue
        if segment.name == "start_to_pre":
            # Try a finite list of task-space staging heights, in configured order.
            replacement = None
            offsets = cfg["grasp_planner"].get(
                "staging_offsets_m", [[0., 0., .05], [0., 0., .10]])
            for offset in offsets:
                offset = np.asarray(offset, float)
                pos = np.asarray(candidate.pre) + offset
                result = _solve_stage(world, candidate, IKStage.PRE_GRASP, pos,
                                      candidate.solutions["pre"], cfg)
                if not result.success:
                    continue
                trial = MotionSegment(
                    "start_to_pre_via_" + "_".join(
                        f"{int(round(v * 1000)):+04d}" for v in offset),
                    MotionPhase.TRANSIT.value,
                    [JointWaypoint("start", direct.start_q),
                     JointWaypoint("staging", np.asarray(result.q)),
                     JointWaypoint("pre", candidate.solutions["pre"])])
                if validate_motion_segment(world.model, world.data, trial, max_step,
                                           target_body_id):
                    replacement = trial
                    break
                candidate.detour_collision_trace.extend(trial.collision_trace)
            if replacement is None:
                return None
            repaired.append(replacement)
        elif segment.name == "pre_to_grasp":
            replacement = _task_space_segment(
                world, candidate, segment.name, MotionPhase.FINAL_APPROACH,
                candidate.pre, candidate.grasp, candidate.solutions["pre"],
                candidate.solutions["grasp"], IKStage.PRE_GRASP, cfg)
            if replacement is None or not validate_motion_segment(
                    world.model, world.data, replacement, max_step, target_body_id):
                if replacement is not None:
                    candidate.detour_collision_trace.extend(replacement.collision_trace)
                return None
            repaired.append(replacement)
        elif segment.name == "grasp_to_lift":
            replacement = _task_space_segment(
                world, candidate, segment.name, MotionPhase.LIFT,
                candidate.grasp, candidate.lift, candidate.solutions["grasp"],
                candidate.solutions["lift"], IKStage.LIFT, cfg)
            if replacement is None or not validate_motion_segment(
                    world.model, world.data, replacement, max_step, target_body_id):
                if replacement is not None:
                    candidate.detour_collision_trace.extend(replacement.collision_trace)
                return None
            repaired.append(replacement)
        else:
            return None
    plan = MotionPlan(direct.start_q.copy(), repaired,
                      [t for s in direct.segments for t in s.collision_trace], True)
    return plan if _validate_motion_plan(world, plan, cfg, target_body_id) else None


def _solve_candidate_stages(world, candidate, start_q, cfg):
    """Solve branches explicitly: both pre and lift are seeded from q_grasp."""
    results = []
    grasp_result = _solve_stage(
        world, candidate, IKStage.GRASP, candidate.grasp, start_q, cfg)
    candidate.ik_results["grasp"] = grasp_result
    if not grasp_result.success:
        return results
    candidate.solutions["grasp"] = np.asarray(grasp_result.q)
    results.append(grasp_result)
    for stage, name, pos in ((IKStage.PRE_GRASP, "pre", candidate.pre),
                             (IKStage.LIFT, "lift", candidate.lift)):
        result = _solve_stage(world, candidate, stage, pos, grasp_result.q, cfg)
        candidate.ik_results[name] = result
        if not result.success:
            break
        candidate.solutions[name] = np.asarray(result.q)
        results.append(result)
    return results


def evaluate_candidates(world, candidates, cfg, excluded=(), target_body_id=None,
                        profile=None):
    table_z = float(cfg["scene"]["table_top_z"])
    min_tcp = float(cfg["grasp_planner"].get("min_tcp_table_clearance", .015))
    warning_margin = float(cfg["grasp_planner"].get("recommended_joint_margin_rad", .05))
    q_now = np.asarray(world.data.qpos[:6], float).copy()
    lo, hi = _hard_limits(world)
    for c in candidates:
        if min(c.pre[2], c.grasp[2], c.lift[2]) - table_z < min_tcp:
            c.rejection_reasons.append("TCP_TABLE_CLEARANCE")
        # GRASP gates the candidate. PRE and LIFT both branch from q_grasp.
        results = _solve_candidate_stages(world, c, q_now, cfg)
        if len(results) != 3:
            c.rejection_reasons.append("IK_BOUNDED_ORIENTATION_UNREACHABLE")
        else:
            c.constraint_level = (ConstraintLevel.EXACT.value if all(
                r.constraint_level == ConstraintLevel.EXACT.value for r in results)
                else ConstraintLevel.RELAXED_ORIENTATION.value)
            c.tcp_error_m = max(r.tcp_error_m for r in results)
            c.approach_error_deg = max(r.approach_error_deg for r in results)
            c.closing_axis_vertical_deg = max(r.closing_axis_error_deg for r in results)
            c.relaxation_penalty = c.approach_error_deg + c.closing_axis_vertical_deg
            qs = [q_now, c.solutions["pre"], c.solutions["grasp"], c.solutions["lift"]]
            c.joint_margin_rad = min(_joint_margin(lo, hi, q) for q in qs[1:])
            if any(np.any(q < lo) or np.any(q > hi) for q in qs[1:]):
                c.rejection_reasons.append("JOINT_OUTSIDE_HARD_LIMITS")
            # A small legal margin is scored/warned, but remains inside hard limits.
            if c.joint_margin_rad < warning_margin:
                c.warnings.append("JOINT_MARGIN_BELOW_RECOMMENDED")
            c.joint_delta_rad = sum(float(np.linalg.norm(b - a))
                                    for a, b in zip(qs, qs[1:]))
            direct = (_profile_motion_plan(world, c, q_now, cfg, profile)
                      if profile is not None and (
                          profile.final_approach_mode == "cartesian" or
                          profile.lift_mode == "cartesian")
                      else _direct_motion_plan(q_now, c))
            if direct is None:
                c.rejection_reasons.append("CARTESIAN_PATH_UNREACHABLE")
                continue
            if profile is not None:
                (c.max_cartesian_deviation_mm,
                 c.lift_horizontal_path_drift_mm) = _path_deviation_metrics(
                    world, c, direct)
            direct_ok = _validate_motion_plan(world, direct, cfg, target_body_id)
            direct_trace = [t for s in direct.segments for t in s.collision_trace]
            c.direct_collision_trace = direct_trace
            c.collision_samples = sum(not t.allowed for t in direct_trace)
            if direct_ok:
                c.motion_plan = direct
            else:
                c.motion_plan = _detour_motion_plan(
                    world, c, direct, cfg, target_body_id)
            if c.motion_plan is None:
                c.rejection_reasons.append("HARD_COLLISION_STATIC_OR_SELF")
        c.feasible = not c.rejection_reasons and len(c.solutions) == 3
    return sorted(candidates, key=lambda c: (
        not c.feasible, c.preference_rank,
        c.constraint_level != ConstraintLevel.EXACT.value,
        c.relaxation_penalty, -c.joint_margin_rad, c.tcp_error_m,
        c.path_length_m, c.joint_delta_rad, c.direction_index))


def plan(center_base, world, cfg, excluded=(), target_body_id=None, profile=None):
    return evaluate_candidates(
        world, generate_candidates(center_base, cfg, profile), cfg, excluded,
        target_body_id, profile)
