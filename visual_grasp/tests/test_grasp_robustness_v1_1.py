import copy

import numpy as np

from multitask.benchmark import generate_xy_grid, generate_yaw_detection_cases
from multitask.grasp_planner import (
    ConstraintLevel,
    MotionPhase,
    CollisionResult,
    ContactCategory,
    action_fingerprint,
    generate_candidates,
    is_failed_action,
    stable_plan_signature,
    contact_allowed,
)


CFG = {
    "scene": {"table_top_z": 0.12},
    "grasp": {"min_grasp_z_clearance": 0.045, "use_table_grasp_z": True,
              "approach_distance": 0.06, "lift_height": 0.15},
    "grasp_planner": {"candidate_count": 8, "start_angle_offset_deg": 0.0},
}


def test_action_fingerprint_ignores_observation_and_absorbs_small_center_noise():
    candidate = generate_candidates(np.array([0.38, -0.11, 0.16]), copy.deepcopy(CFG))[0]
    base = action_fingerprint([0.38, -0.11, 0.16], candidate)
    assert base == action_fingerprint([0.383, -0.11, 0.16], candidate)
    assert stable_plan_signature(1, [0.38, -0.11, 0.16], candidate)[1] == base
    assert stable_plan_signature(2, [0.38, -0.11, 0.16], candidate)[1] == base
    assert base != action_fingerprint([0.392, -0.11, 0.16], candidate)
    candidate.direction_deg += 45
    assert base != action_fingerprint([0.38, -0.11, 0.16], candidate)
    candidate.direction_deg -= 45
    candidate.constraint_level = ConstraintLevel.RELAXED_ORIENTATION.value
    assert base != action_fingerprint([0.38, -0.11, 0.16], candidate)


def test_xy_coverage_grid_has_fixed_yaw_and_25_deterministic_points():
    cases = generate_xy_grid(side=5, yaw_deg=0.0)
    assert len(cases) == 25
    assert len({(c["x"], c["y"]) for c in cases}) == 25
    assert {c["yaw_deg"] for c in cases} == {0.0}


def test_yolo_yaw_cases_repeat_each_xy_without_xy_coupling():
    points = [(0.36, -0.12), (0.40, -0.08)]
    cases = generate_yaw_detection_cases(points)
    assert len(cases) == 16
    for point in points:
        rows = [c for c in cases if (c["x"], c["y"]) == point]
        assert [c["yaw_deg"] for c in rows] == list(range(0, 360, 45))


def test_candidate_report_exposes_bounded_orientation_and_collision_scope():
    candidate = generate_candidates(np.array([0.38, -0.11, 0.16]), copy.deepcopy(CFG))[0]
    report = candidate.public()
    assert report["constraint_level"] == "EXACT"
    assert report["collision_scope"] == "static_and_self"
    assert "approach_error_deg" in report
    assert "closing_axis_vertical_deg" in report


def test_collision_contract_defines_all_motion_phases():
    assert {p.value for p in MotionPhase} == {
        "TRANSIT", "PRE_APPROACH", "FINAL_APPROACH", "CLOSING", "LIFT"}
    result = CollisionResult(True, MotionPhase.CLOSING.value)
    assert result.allowed and result.collision_scope == "static_and_self"
    assert contact_allowed(MotionPhase.FINAL_APPROACH, ContactCategory.TARGET_OBJECT)
    assert contact_allowed(MotionPhase.CLOSING, ContactCategory.TARGET_OBJECT)
    assert not contact_allowed(MotionPhase.TRANSIT, ContactCategory.TARGET_OBJECT)
    assert not contact_allowed(MotionPhase.CLOSING, ContactCategory.STATIC_ENVIRONMENT)
    assert not contact_allowed(MotionPhase.LIFT, ContactCategory.NON_ADJACENT_SELF)


def test_failed_action_is_blocked_until_object_moves_more_than_10mm():
    candidate = generate_candidates(np.array([0.38, -0.11, 0.16]), copy.deepcopy(CFG))[0]
    failed = [{"center": np.array([0.38, -0.11, 0.16]),
               "fingerprint": action_fingerprint([0.38, -0.11, 0.16], candidate)}]
    assert is_failed_action([0.386, -0.11, 0.16], candidate, failed)
    assert not is_failed_action([0.391, -0.11, 0.16], candidate, failed)
