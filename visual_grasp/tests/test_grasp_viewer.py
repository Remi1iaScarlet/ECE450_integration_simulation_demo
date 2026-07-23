import mujoco
import numpy as np

from multitask.grasp_viewer import (
    REPRESENTATIVE_POSITIONS,
    ViewerCase,
    build_viewer_cases,
    parse_args,
    reset_demo_scene,
    validate_cases,
)
from multitask.grasp_heatmap import _configured_world


def test_representative_viewer_cases_are_fixed_and_include_weak_cell():
    cases = build_viewer_cases("representative", yaw_deg=15.0)
    assert len(cases) == 5
    assert tuple((case.x_m, case.y_m) for case in cases) == REPRESENTATIVE_POSITIONS
    assert (cases[0].x_m, cases[0].y_m) == (0.34, -0.16)
    assert all(case.yaw_deg == 15.0 for case in cases)


def test_full_grid_matches_five_by_four_e2e_domain():
    cases = build_viewer_cases("full-grid")
    assert len(cases) == 20
    assert len({case.x_m for case in cases}) == 5
    assert len({case.y_m for case in cases}) == 4


def test_custom_positions_override_preset():
    cases = build_viewer_cases("full-grid", ((0.38, -0.07),), yaw_deg=30)
    assert len(cases) == 1
    assert cases[0].case_id == "custom-01"
    assert (cases[0].x_m, cases[0].y_m, cases[0].yaw_deg) == (0.38, -0.07, 30)


def test_viewer_cases_are_rejected_outside_operating_domain():
    cases = build_viewer_cases("representative", ((0.50, -0.07),))
    try:
        validate_cases(cases, (0.30, 0.46), (-0.22, 0.02))
    except ValueError as exc:
        assert "outside the operating domain" in str(exc)
    else:
        raise AssertionError("expected out-of-domain position to be rejected")


def test_unknown_preset_is_rejected():
    try:
        build_viewer_cases("unknown")
    except ValueError as exc:
        assert "unknown viewer preset" in str(exc)
    else:
        raise AssertionError("expected unknown preset to be rejected")


def test_world_after_step_hook_is_optional_and_called_once_per_step():
    _, world, _ = _configured_world("sim_gt", "piper_real", "radial_single")
    calls = []
    world.after_step = lambda: calls.append(world.data.time)
    world.settle(3)
    assert len(calls) == 3
    assert calls == sorted(calls)
    world.after_step = None


def test_viewer_defaults_to_demo_scene_and_continuous_motion():
    options = parse_args([])
    assert options.scene_layout == "demo"
    assert options.motion_mode == "continuous"


def test_clearance_metric_exposes_old_scan_collision_and_safe_pose():
    _, world, _ = _configured_world("sim_gt", "piper_real", "radial_single")
    world.data.qpos[:6] = [0.15, 0.5, 0.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(world.model, world.data)
    old_clearance = min(
        row.table_clearance_m for row in world.measure_table_clearance("OLD_SCAN"))
    world.data.qpos[:6] = [-0.05, 0.5, 0.0, 0.0, -0.4, 0.0]
    mujoco.mj_forward(world.model, world.data)
    safe_clearance = min(
        row.table_clearance_m for row in world.measure_table_clearance("SAFE_SCAN"))
    assert old_clearance < 0.0
    assert safe_clearance >= 0.010


def test_demo_reset_preserves_three_requested_scene_objects():
    cfg, world, _ = _configured_world("sim_gt", "piper_real", "radial_single")
    reset_demo_scene(world, cfg, ViewerCase("demo", 0.38, -0.07, 0.0))
    assert np.allclose(world.body_pos("ycb_bottle")[:2], [0.38, -0.07], atol=0.002)
    assert world.body_pos("demo_bottle")[2] >= cfg["scene"]["table_top_z"]
    assert world.body_pos("demo_box") is not None
    bowl = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, "bowl")
    bowl_geoms = np.flatnonzero(world.model.geom_bodyid == bowl)
    assert all(world.model.geom_rgba[geom_id, 3] == 0 for geom_id in bowl_geoms)
