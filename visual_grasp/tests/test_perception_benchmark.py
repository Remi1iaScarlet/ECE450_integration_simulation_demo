from multitask.perception_benchmark import (
    DetectionObservation, PerceptionBenchmarkSpec, evaluate_threshold,
    reevaluate_calibration)


def _row(case_id, case_class, label=None, confidence=None, predicted=None,
         actual=(0.38, -0.07, 0.12)):
    return DetectionObservation(
        case_id=case_id, case_class=case_class, scan_pose_index=0,
        target_present=case_class != "NEGATIVE",
        actual_center_base=actual if case_class != "NEGATIVE" else None,
        predicted_label=label, confidence=confidence,
        predicted_center_base=predicted, render_detect_ms=10.0)


def test_threshold_evaluation_is_case_level_across_scan_poses():
    rows = [
        _row("p1", "INTERIOR", "bottle", 0.20, (0.381, -0.069, 0.17)),
        DetectionObservation("p1", "INTERIOR", 1, True, (0.38, -0.07, 0.12),
                             "bottle", 0.40, (0.38, -0.07, 0.17), 12.0),
        _row("p2", "INTERIOR"),
        _row("n1", "NEGATIVE", "bottle", 0.15, (0.4, -0.1, 0.17)),
    ]
    metrics = evaluate_threshold(rows, 0.30)
    assert metrics.positive_cases == 2
    assert metrics.detected_cases == 1
    assert metrics.recall_pct == 50.0
    assert metrics.false_positive_cases == 0
    assert metrics.xy_error_p95_mm == 0.0


def test_threshold_gate_requires_recall_fpr_and_xy_accuracy():
    rows = []
    for index in range(20):
        rows.append(_row(
            f"p{index}", "INTERIOR", "bottle", 0.25,
            (0.381, -0.07, 0.17)))
    for index in range(50):
        rows.append(_row(f"n{index}", "NEGATIVE"))
    metrics = evaluate_threshold(rows, 0.20)
    assert metrics.recall_pct == 100.0
    assert metrics.false_positive_rate_pct == 0.0
    assert metrics.xy_error_p95_mm == 1.0
    assert metrics.gate_passed


def test_default_thresholds_include_selected_calibration_candidate():
    assert PerceptionBenchmarkSpec().confidence_thresholds == (
        0.05, 0.075, 0.10, 0.15, 0.20, 0.25, 0.30)


def test_offline_reevaluation_selects_highest_passing_threshold(tmp_path):
    rows = []
    for index in range(20):
        confidence = 0.08 if index < 2 else 0.20
        rows.append(_row(
            f"p{index}", "INTERIOR", "bottle", confidence,
            (0.381, -0.07, 0.17)))
    for index in range(50):
        rows.append(_row(f"n{index}", "NEGATIVE"))
    csv_path = tmp_path / "perception_observations.csv"
    import csv
    from dataclasses import asdict
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(rows[0])))
        writer.writeheader()
        writer.writerows(asdict(row) for row in rows)
    summary = reevaluate_calibration(
        csv_path, tmp_path / "result", thresholds=(0.075, 0.10))
    assert summary["gate"]["passed"]
    assert summary["gate"]["selected_threshold"] == 0.075
    assert (tmp_path / "result" / "perception_threshold_sweep.csv").exists()
    assert (tmp_path / "result" / "failure_montage_offline.png").exists()
