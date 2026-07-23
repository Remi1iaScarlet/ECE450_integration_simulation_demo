from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from multitask import grasp_planner, primitives
from multitask.executor import TaskExecutor
from multitask.grasp_planner import (GraspProfile,
                                     build_cartesian_segment, generate_candidates,
                                     resolve_grasp_profile,
                                     resolve_grasp_z,
                                     select_preferred_directions)
from multitask.motion_plan import (JointWaypoint, MotionPhase, MotionPlan,
                                   MotionSegment)


CFG = {
    "scene": {"table_top_z": 0.12},
    "grasp": {"min_grasp_z_clearance": 0.045, "use_table_grasp_z": True,
              "approach_distance": 0.06, "lift_height": 0.15,
              "strategy": "multi_candidate_v1_1"},
    "grasp_planner": {"candidate_count": 8, "start_angle_offset_deg": 0.0,
                      "plan_start_tolerance_rad": 0.03},
}


def test_lift_and_pre_are_both_seeded_from_grasp_solution():
    candidate = generate_candidates([0.38, -0.11, 0.16], CFG)[0]
    calls = []

    def fake_solve(world, candidate, stage, pos, seed, cfg):
        calls.append((stage.value, np.asarray(seed).copy()))
        q = np.full(6, {"GRASP": 1.0, "PRE_GRASP": 2.0, "LIFT": 3.0}[stage.value])
        return SimpleNamespace(success=True, q=q)

    with patch("multitask.grasp_planner._solve_stage", side_effect=fake_solve):
        results = grasp_planner._solve_candidate_stages(
            object(), candidate, np.zeros(6), CFG)
    assert len(results) == 3
    assert np.array_equal(calls[1][1], np.ones(6))
    assert np.array_equal(calls[2][1], np.ones(6))


def test_motion_plan_public_contains_start_segments_and_waypoints():
    assert grasp_planner.JointWaypoint is JointWaypoint
    assert grasp_planner.MotionSegment is MotionSegment
    assert grasp_planner.MotionPlan is MotionPlan
    assert grasp_planner.MotionPhase is MotionPhase
    plan = MotionPlan(np.zeros(6), [MotionSegment(
        "start_to_pre", MotionPhase.TRANSIT.value,
        [JointWaypoint("start", np.zeros(6)), JointWaypoint("pre", np.ones(6))])])
    report = plan.public()
    assert report["start_q"] == [0.0] * 6
    assert report["segments"][0]["waypoints"][1]["name"] == "pre"


def test_executor_rejects_motion_plan_start_mismatch_without_motion():
    world = SimpleNamespace(data=SimpleNamespace(qpos=np.ones(7)), model=None)
    executor = TaskExecutor(world, CFG, verbose=False)
    plan = MotionPlan(np.zeros(6), [])
    ctx = {"selected_candidate": SimpleNamespace(motion_plan=plan),
           "pick_plan": None}
    result = executor._st_execute_pick(ctx)
    assert not result["success"]
    assert result["data"]["reason"] == "PLAN_START_STATE_MISMATCH"


def test_executor_primitive_runs_exact_validated_waypoints_without_ik():
    class World:
        def __init__(self):
            self.data = SimpleNamespace(qpos=np.zeros(7))
            self.commands = []

        def set_arm(self, q, grip=None, steps=0):
            self.commands.append(np.asarray(q).copy())
            self.data.qpos[:6] = q

        def gripper(self, value, steps):
            pass

        def tcp_pos(self):
            return np.zeros(3)

    plan = MotionPlan(np.zeros(6), [
        MotionSegment("start_to_pre", MotionPhase.TRANSIT.value,
                      [JointWaypoint("start", np.zeros(6)),
                       JointWaypoint("stage", np.ones(6)),
                       JointWaypoint("pre", np.full(6, 2.0))]),
        MotionSegment("pre_to_grasp", MotionPhase.FINAL_APPROACH.value,
                      [JointWaypoint("pre", np.full(6, 2.0)),
                       JointWaypoint("grasp", np.full(6, 3.0))]),
        MotionSegment("grasp_to_lift", MotionPhase.LIFT.value,
                      [JointWaypoint("grasp", np.full(6, 3.0)),
                       JointWaypoint("lift", np.full(6, 4.0))]),
    ])
    world = World()
    with patch("multitask.primitives.grasp_sim.GRIP_OPEN", 0.03), \
            patch("multitask.primitives.grasp_sim.GRIP_CLOSE", -0.01):
        result = primitives.execute_motion_plan(world, plan)
    assert result["success"]
    assert [q[0] for q in world.commands] == [1.0, 2.0, 3.0, 4.0]


def test_continuous_executor_passes_each_segment_as_one_waypoint_polyline():
    class World:
        def __init__(self):
            self.model = SimpleNamespace(opt=SimpleNamespace(timestep=0.002))
            self.data = SimpleNamespace(qpos=np.zeros(7))
            self.calls = []

        def follow_joint_waypoints(self, waypoints, **kwargs):
            self.calls.append((tuple(np.asarray(q).copy() for q in waypoints), kwargs))
            return SimpleNamespace(
                steps=20, max_following_error_rad=0.01,
                min_table_clearance_m=0.05)

        def gripper(self, value, steps):
            self.gripper_command = (value, steps)

        def tcp_pos(self):
            return np.zeros(3)

    plan = MotionPlan(np.zeros(6), [
        MotionSegment("start_to_pre", MotionPhase.TRANSIT.value,
                      [JointWaypoint("start", np.zeros(6)),
                       JointWaypoint("stage", np.ones(6)),
                       JointWaypoint("pre", np.full(6, 2.0))]),
        MotionSegment("pre_to_grasp", MotionPhase.FINAL_APPROACH.value,
                      [JointWaypoint("pre", np.full(6, 2.0)),
                       JointWaypoint("grasp", np.full(6, 3.0))]),
        MotionSegment("grasp_to_lift", MotionPhase.LIFT.value,
                      [JointWaypoint("grasp", np.full(6, 3.0)),
                       JointWaypoint("lift", np.full(6, 4.0))]),
    ])
    world = World()
    config = primitives.TrajectoryExecutionConfig()
    result = primitives.execute_continuous_motion_plan(world, plan, config)
    assert result["success"]
    assert len(world.calls) == 3
    assert len(world.calls[0][0]) == 3
    assert world.calls[0][1]["final_settle_steps"] == 0
    assert world.calls[1][1]["final_settle_steps"] == config.final_settle_steps


def test_bottle_profile_prefers_absolute_directions_without_changing_default_order():
    cfg = {**CFG, "grasp": {**CFG["grasp"], "profiles": {"bottle": {
        "strategy": "multi_candidate_v1_1",
        "preferred_directions_deg": [0.0, 315.0],
        "final_approach_mode": "cartesian", "lift_mode": "cartesian",
        "cartesian_waypoint_count": 3}}}}
    profile = resolve_grasp_profile(cfg, "bottle")
    candidates = generate_candidates([0.38, -0.11, 0.16], cfg, profile)
    assert [candidate.direction_deg for candidate in candidates[:2]] == [0.0, 315.0]
    assert len({round(candidate.direction_deg, 6) for candidate in candidates}) == 8
    default = generate_candidates([0.38, -0.11, 0.16], cfg)
    angles = np.unwrap(np.deg2rad([candidate.direction_deg for candidate in default]))
    assert np.allclose(np.diff(angles), np.deg2rad(45.0))


def test_lateral_band_policy_uses_radial_offset_below_split_and_absolute_above():
    cfg = {**CFG, "grasp": {**CFG["grasp"], "profiles": {"bottle": {
        "strategy": "multi_candidate_v1_1",
        "direction_policy": {
            "mode": "lateral_band", "split_y_m": -0.08,
            "lower_band": {"reference": "radial", "offset_deg": -15.0},
            "upper_band": {"reference": "absolute", "angle_deg": 0.0}},
        "fallback_directions_deg": [315.0]}}}}
    profile = resolve_grasp_profile(cfg, "bottle")
    lower = select_preferred_directions([0.4, -0.115, 0.16], profile)
    upper = select_preferred_directions([0.4, -0.025, 0.16], profile)
    assert np.isclose(lower[0], (np.rad2deg(np.arctan2(-0.115, 0.4)) - 15) % 360)
    assert lower[1] == 315.0
    assert upper == (0.0, 315.0)
    candidates = generate_candidates([0.4, -0.115, 0.16], cfg, profile)
    assert candidates[0].direction_policy_band == "lower"
    assert np.isclose(candidates[0].direction_reference_deg,
                      np.rad2deg(np.arctan2(-0.115, 0.4)) % 360)


def test_bottle_grasp_height_override_does_not_change_default_candidates():
    cfg = {**CFG, "grasp": {**CFG["grasp"], "profiles": {"bottle": {
        "strategy": "multi_candidate_v1_1", "grasp_z_clearance_m": 0.075}}}}
    profile = resolve_grasp_profile(cfg, "bottle")
    assert np.isclose(resolve_grasp_z(
        [0.38, -0.07, 0.16], 0.12, 0.045, profile), 0.195)
    bottle = generate_candidates([0.38, -0.07, 0.16], cfg, profile)[0]
    default = generate_candidates([0.38, -0.07, 0.16], cfg)[0]
    assert np.isclose(bottle.grasp[2], 0.195)
    assert np.isclose(bottle.grasp_z_clearance_m, 0.075)
    assert np.isclose(default.grasp[2], 0.165)


def test_cartesian_segment_has_bounded_intermediate_waypoints():
    candidate = generate_candidates([0.38, -0.11, 0.16], CFG)[0]
    calls = []

    def fake_solve(world, candidate, stage, pos, seed, cfg):
        calls.append((stage.value, np.asarray(pos).copy()))
        return SimpleNamespace(success=True, q=np.full(6, len(calls)))

    with patch("multitask.grasp_planner._solve_stage", side_effect=fake_solve):
        segment = build_cartesian_segment(
            object(), candidate, "pre_to_grasp", MotionPhase.FINAL_APPROACH,
            candidate.pre, candidate.grasp, np.zeros(6), np.ones(6), 3, CFG)
    assert segment is not None
    assert len(segment.waypoints) == 5
    assert [call[0] for call in calls] == ["GRASP"] * 3
    expected = [candidate.pre + alpha * (candidate.grasp - candidate.pre)
                for alpha in (0.25, 0.5, 0.75)]
    assert all(np.allclose(call[1], point) for call, point in zip(calls, expected))
