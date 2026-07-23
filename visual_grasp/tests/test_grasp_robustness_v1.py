import copy

import numpy as np

from multitask.benchmark import generate_cases
from multitask.grasp_planner import generate_candidates


CFG = {
    "scene": {"table_top_z": 0.12},
    "grasp": {
        "min_grasp_z_clearance": 0.045,
        "use_table_grasp_z": True,
        "approach_distance": 0.06,
        "lift_height": 0.15,
    },
    "grasp_planner": {"candidate_count": 8, "start_angle_offset_deg": 0.0},
}


def test_cases_are_seeded_and_include_full_yaw_range():
    a = generate_cases(10, 20260712, (0.34, 0.42), (-0.16, -0.06), (0, 360))
    b = generate_cases(10, 20260712, (0.34, 0.42), (-0.16, -0.06), (0, 360))
    c = generate_cases(10, 7, (0.34, 0.42), (-0.16, -0.06), (0, 360))
    assert a == b and a != c
    assert all(0 <= row["yaw_deg"] < 360 for row in a)


def test_generates_eight_distinct_radial_candidates_without_mutating_center():
    center = np.array([0.38, -0.11, 0.16])
    before = center.copy()
    candidates = generate_candidates(center, copy.deepcopy(CFG))
    assert len(candidates) == 8
    angles = np.unwrap(np.deg2rad([c.direction_deg for c in candidates]))
    assert np.allclose(np.diff(angles), np.deg2rad(45.0))
    assert np.isclose(candidates[0].direction_deg,
                      np.rad2deg(np.arctan2(center[1], center[0])) % 360)
    assert len({c.candidate_id for c in candidates}) == 8
    assert np.array_equal(center, before)
    assert all(np.isclose(c.grasp[2], 0.165) for c in candidates)
    assert all(np.isclose(np.linalg.norm(c.pre[:2] - c.grasp[:2]), 0.06)
               for c in candidates)
