import json

from multitask.grasp_heatmap import (
    DOMAIN_IN,
    DOMAIN_OUT,
    GraspGridSpec,
    GraspTrialCase,
    GraspTrialRecord,
    DirectionProbeResult,
    _configured_world,
    generate_grasp_grid_cases,
    generate_ood_diagnostic_cases,
    reset_target_for_trial,
    summarize_backend,
    summarize_direction_probe,
)


def test_grid_cases_are_seeded_clipped_and_repeat_each_cell():
    spec = GraspGridSpec(x_values_m=(0.34, 0.42), y_values_m=(-0.16, 0.02),
                         repeats_per_cell=3, xy_jitter_m=0.003, seed=7)
    first = generate_grasp_grid_cases(spec, (0.30, 0.46), (-0.22, 0.02))
    second = generate_grasp_grid_cases(spec, (0.30, 0.46), (-0.22, 0.02))
    assert first == second
    assert len(first) == 12
    assert {case.domain_class for case in first} == {DOMAIN_IN}
    assert all(-0.22 <= case.actual_y_m <= 0.02 for case in first)
    assert all(0.30 <= case.actual_x_m <= 0.46 for case in first)
    assert all(case.actual_y_m < 0.02 for case in first if case.grid_y_m == 0.02)


def test_out_of_domain_cases_are_explicit_diagnostics():
    cases = generate_ood_diagnostic_cases((0.34, 0.42), (0.05, 0.12))
    assert len(cases) == 4
    assert {case.domain_class for case in cases} == {DOMAIN_OUT}
    assert all(case.repeat_index == 1 for case in cases)


def _record(case_id, x, y, success, domain=DOMAIN_IN):
    return GraspTrialRecord(
        case_id=case_id, backend="sim_gt", target_label="bottle",
        grid_x_m=x, grid_y_m=y, actual_x_m=x, actual_y_m=y,
        yaw_deg=0.0, repeat_index=1, domain_class=domain,
        success=success,
        initial_failed_state="" if success else "VERIFY_GRASP",
        initial_failure_reason="" if success else "LIFT_THRESHOLD_NOT_MET",
        failed_state="" if success else "VERIFY_GRASP",
        failure_reason="" if success else "LIFT_THRESHOLD_NOT_MET",
        lift_height_mm=100.0 if success else 2.0, task_duration_ms=10.0,
        detect_source_ms=1.0, execute_pick_ms=8.0,
        source_center_error_mm=0.0, selected_direction_deg=None,
        direction_policy_mode=None, direction_policy_band=None,
        direction_reference_deg=None,
        planned_grasp_z_m=None, grasp_z_clearance_m=None,
        max_cartesian_deviation_mm=None, lift_horizontal_path_drift_mm=None,
        execution_trace_json="null", target_settle_displacement_mm=0.0,
        unexpected_contacts_json=json.dumps([]), state_log_json=json.dumps([]))


def test_summary_excludes_ood_and_reports_cell_wilson_interval():
    records = [
        _record("a", 0.34, -0.16, True),
        _record("b", 0.34, -0.16, False),
        _record("ood", 0.34, 0.12, False, DOMAIN_OUT),
    ]
    summary = summarize_backend(records)
    assert summary["trials"] == 2
    assert summary["successes"] == 1
    assert summary["success_rate_pct"] == 50.0
    assert len(summary["cells"]) == 1
    assert summary["cells"][0]["wilson_95_pct"] is not None
    assert summary["failure_reason_counts"] == {"LIFT_THRESHOLD_NOT_MET": 1}


def test_direction_probe_classifies_selection_execution_and_reachability():
    rows = [
        DirectionProbeResult(0.34, -0.16, 0, True, True, False, 2, "LIFT"),
        DirectionProbeResult(0.34, -0.16, 45, True, True, True, 40, ""),
        DirectionProbeResult(0.36, -0.16, 0, True, True, False, 2, "LIFT"),
        DirectionProbeResult(0.38, -0.16, 0, False, False, False, None, "IK"),
    ]
    summary = summarize_direction_probe(rows)
    classes = {(row["grid_x_m"], row["grid_y_m"]): row["classification"]
               for row in summary["cells"]}
    assert classes[(0.34, -0.16)] == "DIRECTION_SELECTION"
    assert classes[(0.36, -0.16)] == "EXECUTION_OR_CONTACT"
    assert classes[(0.38, -0.16)] == "REACHABILITY_OR_COLLISION"


def test_pick_scene_isolation_removes_bowl_and_preserves_target_xy():
    cfg, world, _ = _configured_world("sim_gt", "piper_real", "radial_single")
    case = GraspTrialCase(
        case_id="isolation", grid_x_m=0.42, grid_y_m=0.02,
        actual_x_m=0.42, actual_y_m=0.02, yaw_deg=0.0,
        repeat_index=1, domain_class=DOMAIN_IN)
    report = reset_target_for_trial(world, cfg, "bottle", case)
    assert "bowl" in report.hidden_bodies
    assert report.disabled_geom_ids
    assert report.target_settle_displacement_mm <= 5.0
    assert report.unexpected_contacts == ()
