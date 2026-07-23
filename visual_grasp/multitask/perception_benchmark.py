"""Perception-only YOLO/RGB-D calibration and gated interior validation."""
from __future__ import annotations

import csv
import ast
import json
import pathlib
import statistics
import time
from collections import defaultdict
from dataclasses import asdict, dataclass
from typing import Sequence

import mujoco
import numpy as np
from PIL import Image

from .config import load_config
from .grasp_heatmap import (
    DEFAULT_X_VALUES, DOMAIN_IN, GraspGridSpec, GraspTrialCase,
    _configured_world, _domain_ranges, _move_body, _target_body_name,
    aggregate_grasp_heatmap, generate_grasp_grid_cases, reset_target_for_trial,
    run_grasp_grid)
from .object_registry import collect_yolo_candidates


DEFAULT_INTERIOR_Y = (-0.16, -0.115, -0.07, -0.025, 0.01)
E2E_INTERIOR_Y = (-0.16, -0.115, -0.07, -0.025)


@dataclass(frozen=True)
class PerceptionBenchmarkSpec:
    target_label: str = "bottle"
    confidence_thresholds: tuple[float, ...] = (
        0.05, 0.075, 0.10, 0.15, 0.20, 0.25, 0.30)
    positive_repeats: int = 10
    negative_cases: int = 50
    interior_y_max_m: float = 0.01
    boundary_y_m: float = 0.02
    seed: int = 20260713
    model: str = "piper_real"


@dataclass
class DetectionObservation:
    case_id: str
    case_class: str
    scan_pose_index: int
    target_present: bool
    actual_center_base: tuple[float, float, float] | None
    predicted_label: str | None
    confidence: float | None
    predicted_center_base: tuple[float, float, float] | None
    render_detect_ms: float


@dataclass(frozen=True)
class PerceptionMetrics:
    threshold: float
    positive_cases: int
    detected_cases: int
    recall_pct: float
    negative_cases: int
    false_positive_cases: int
    false_positive_rate_pct: float
    xy_error_mean_mm: float | None
    xy_error_p95_mm: float | None
    z_bias_mean_mm: float | None
    interior_odd_rejection_rate_pct: float | None
    mean_scan_duration_ms: float | None
    gate_passed: bool


def _write_dataclass_csv(path: pathlib.Path, rows: Sequence):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(rows[0])))
        writer.writeheader()
        writer.writerows(asdict(row) for row in rows)


def _candidate_for_case(rows, threshold, target_label):
    candidates = [row for row in rows
                  if row.predicted_label == target_label
                  and row.confidence is not None
                  and row.confidence >= threshold
                  and row.predicted_center_base is not None]
    return max(candidates, key=lambda row: row.confidence) if candidates else None


def evaluate_threshold(observations: Sequence[DetectionObservation], threshold: float,
                       target_label="bottle", domain_x=(0.30, 0.46),
                       domain_y=(-0.22, 0.02)) -> PerceptionMetrics:
    grouped = defaultdict(list)
    for row in observations:
        grouped[row.case_id].append(row)
    positive_ids = [case_id for case_id, rows in grouped.items()
                    if rows[0].case_class == "INTERIOR"]
    negative_ids = [case_id for case_id, rows in grouped.items()
                    if rows[0].case_class == "NEGATIVE"]
    selected = {case_id: _candidate_for_case(grouped[case_id], threshold, target_label)
                for case_id in positive_ids}
    detected = [row for row in selected.values() if row is not None]
    xy_errors, z_biases, ood = [], [], 0
    for row in detected:
        actual = np.asarray(row.actual_center_base, float)
        predicted = np.asarray(row.predicted_center_base, float)
        xy_errors.append(float(np.linalg.norm(predicted[:2] - actual[:2]) * 1000.0))
        z_biases.append(float((predicted[2] - actual[2]) * 1000.0))
        if not (domain_x[0] <= predicted[0] <= domain_x[1] and
                domain_y[0] <= predicted[1] <= domain_y[1]):
            ood += 1
    false_positives = sum(
        _candidate_for_case(grouped[case_id], threshold, target_label) is not None
        for case_id in negative_ids)
    recall = 100.0 * len(detected) / len(positive_ids) if positive_ids else 0.0
    fpr = 100.0 * false_positives / len(negative_ids) if negative_ids else 0.0
    unique_durations = {}
    for row in observations:
        unique_durations[(row.case_id, row.scan_pose_index)] = row.render_detect_ms
    xy_p95 = float(np.percentile(xy_errors, 95)) if xy_errors else None
    gate = bool(recall >= 95.0 and fpr <= 2.0 and
                xy_p95 is not None and xy_p95 <= 10.0)
    return PerceptionMetrics(
        threshold=round(float(threshold), 4), positive_cases=len(positive_ids),
        detected_cases=len(detected), recall_pct=round(recall, 2),
        negative_cases=len(negative_ids), false_positive_cases=false_positives,
        false_positive_rate_pct=round(fpr, 2),
        xy_error_mean_mm=(round(statistics.fmean(xy_errors), 3)
                          if xy_errors else None),
        xy_error_p95_mm=round(xy_p95, 3) if xy_p95 is not None else None,
        z_bias_mean_mm=(round(statistics.fmean(z_biases), 3)
                        if z_biases else None),
        interior_odd_rejection_rate_pct=(round(100.0 * ood / len(detected), 2)
                                         if detected else None),
        mean_scan_duration_ms=(round(statistics.fmean(unique_durations.values()), 2)
                               if unique_durations else None),
        gate_passed=gate)


def _boundary_metrics(observations, threshold, target_label, domain_x, domain_y):
    grouped = defaultdict(list)
    for row in observations:
        if row.case_class == "BOUNDARY":
            grouped[row.case_id].append(row)
    selected = [_candidate_for_case(rows, threshold, target_label)
                for rows in grouped.values()]
    detected = [row for row in selected if row is not None]
    xy_errors, ood = [], 0
    for row in detected:
        actual = np.asarray(row.actual_center_base, float)
        predicted = np.asarray(row.predicted_center_base, float)
        xy_errors.append(float(np.linalg.norm(predicted[:2] - actual[:2]) * 1000.0))
        ood += not (domain_x[0] <= predicted[0] <= domain_x[1] and
                    domain_y[0] <= predicted[1] <= domain_y[1])
    return {
        "cases": len(selected), "detected_cases": len(detected),
        "detection_rate_pct": round(100.0 * len(detected) / len(selected), 2)
        if selected else None,
        "odd_rejection_cases": int(ood),
        "odd_rejection_rate_pct": round(100.0 * ood / len(detected), 2)
        if detected else None,
        "xy_error_p95_mm": round(float(np.percentile(xy_errors, 95)), 3)
        if xy_errors else None,
    }


def _benchmark_cases(spec, domain_x, domain_y):
    interior_spec = GraspGridSpec(
        target_label=spec.target_label, x_values_m=DEFAULT_X_VALUES,
        y_values_m=DEFAULT_INTERIOR_Y, repeats_per_cell=spec.positive_repeats,
        xy_jitter_m=0.003, yaw_range_deg=(0.0, 360.0), seed=spec.seed,
        model=spec.model, backends=("yolo",))
    interior = generate_grasp_grid_cases(interior_spec, domain_x, domain_y)
    rng = np.random.default_rng(spec.seed + 1)
    boundary = []
    for xi, x in enumerate(DEFAULT_X_VALUES):
        for repeat in range(spec.positive_repeats):
            actual_x = float(np.clip(x + rng.uniform(-0.003, 0.003),
                                     domain_x[0] + 1e-6, domain_x[1] - 1e-6))
            boundary.append(GraspTrialCase(
                case_id=f"boundary-x{xi + 1:02d}-r{repeat + 1:02d}",
                grid_x_m=x, grid_y_m=spec.boundary_y_m,
                actual_x_m=actual_x, actual_y_m=spec.boundary_y_m,
                yaw_deg=float(rng.uniform(0.0, 360.0)), repeat_index=repeat + 1,
                domain_class=DOMAIN_IN))
    return interior, boundary


def _collect_observations(spec, domain_x, domain_y):
    cfg, world, canonical = _configured_world("yolo", spec.model, "radial_single")
    scan_poses = cfg["detector"]["scan_poses"]
    min_conf = min(spec.confidence_thresholds + (0.05,))
    interior, boundary = _benchmark_cases(spec, domain_x, domain_y)
    observations, thumbnails = [], {}
    cases = [(case, "INTERIOR") for case in interior]
    cases += [(case, "BOUNDARY") for case in boundary]
    total = len(cases) + spec.negative_cases
    target_body = _target_body_name(cfg, spec.target_label)

    def scan_case(case_id, case_class, target_present, actual):
        first_rgb = None
        for pose_index, ctrl in enumerate(scan_poses):
            world.move_to_pose(np.asarray(ctrl, float))
            started = time.perf_counter()
            candidates, rgb = collect_yolo_candidates(
                world, cfg["objects"]["graspable"], cfg["objects"]["containers"],
                min_conf=min_conf, table_z=cfg["scene"]["table_top_z"],
                radius_factor=cfg["detector"].get("radius_correction", 1.0))
            elapsed = (time.perf_counter() - started) * 1000.0
            if first_rgb is None:
                first_rgb = np.asarray(Image.fromarray(rgb).resize((240, 180))).copy()
            if not candidates:
                observations.append(DetectionObservation(
                    case_id, case_class, pose_index, target_present, actual,
                    None, None, None, round(elapsed, 2)))
            for candidate in candidates:
                observations.append(DetectionObservation(
                    case_id, case_class, pose_index, target_present, actual,
                    candidate["label"], float(candidate["confidence"]),
                    tuple(float(v) for v in candidate["center_base"]),
                    round(elapsed, 2)))
        if case_class == "INTERIOR":
            thumbnails[case_id] = first_rgb

    for index, (case, case_class) in enumerate(cases, start=1):
        reset_target_for_trial(world, cfg, spec.target_label, case)
        actual = tuple(float(v) for v in world.body_pos(target_body))
        scan_case(case.case_id, case_class, True, actual)
        print(f"[{canonical}/perception] {index:03d}/{total}: {case_class} {case.case_id}")

    base_case = interior[0]
    for negative_index in range(spec.negative_cases):
        reset_target_for_trial(world, cfg, spec.target_label, base_case)
        _move_body(world, target_body, (2.0, 2.0, 0.20))
        mujoco.mj_forward(world.model, world.data)
        case_id = f"negative-{negative_index + 1:03d}"
        scan_case(case_id, "NEGATIVE", False, None)
        print(f"[{canonical}/perception] {len(cases) + negative_index + 1:03d}/"
              f"{total}: NEGATIVE {case_id}")
    return observations, thumbnails


def _render_threshold_plot(metrics, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    thresholds = [row.threshold for row in metrics]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    axes[0].plot(thresholds, [row.recall_pct for row in metrics], marker="o",
                 label="Recall")
    axes[0].axhline(95.0, color="gray", linestyle="--", label="95% Gate")
    axes[0].set(xlabel="Confidence threshold", ylabel="Recall (%)", ylim=(0, 101))
    axes[0].legend()
    axes[1].plot(thresholds, [row.false_positive_rate_pct for row in metrics],
                 marker="s", label="Empty-table false positives")
    axes[1].axhline(2.0, color="gray", linestyle="--", label="2% ceiling")
    axes[1].set(xlabel="Confidence threshold", ylabel="False-positive rate (%)",
                ylim=(0, max(5.0, max(row.false_positive_rate_pct for row in metrics) + 2)))
    axes[1].legend()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _render_failure_montage(observations, thumbnails, threshold, target_label, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    grouped = defaultdict(list)
    for row in observations:
        if row.case_class == "INTERIOR":
            grouped[row.case_id].append(row)
    misses = [case_id for case_id, rows in grouped.items()
              if _candidate_for_case(rows, threshold, target_label) is None][:12]
    fig, axes = plt.subplots(3, 4, figsize=(10, 7), constrained_layout=True)
    for axis, case_id in zip(axes.flat, misses):
        axis.imshow(thumbnails[case_id])
        confidences = [row.confidence for row in grouped[case_id]
                       if row.predicted_label == target_label and row.confidence is not None]
        axis.set_title(f"{case_id}\nmax={max(confidences):.3f}" if confidences
                       else f"{case_id}\nno candidate", fontsize=8)
        axis.axis("off")
    for axis in list(axes.flat)[len(misses):]:
        axis.axis("off")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _read_observations_csv(path: pathlib.Path) -> list[DetectionObservation]:
    def optional_tuple(value):
        if not value:
            return None
        parsed = ast.literal_eval(value)
        if not isinstance(parsed, tuple) or len(parsed) != 3:
            raise ValueError(f"expected a three-element tuple, got {value!r}")
        return tuple(float(item) for item in parsed)

    observations = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            observations.append(DetectionObservation(
                case_id=row["case_id"], case_class=row["case_class"],
                scan_pose_index=int(row["scan_pose_index"]),
                target_present=row["target_present"].strip().lower() == "true",
                actual_center_base=optional_tuple(row["actual_center_base"]),
                predicted_label=row["predicted_label"] or None,
                confidence=(float(row["confidence"])
                            if row["confidence"] else None),
                predicted_center_base=optional_tuple(row["predicted_center_base"]),
                render_detect_ms=float(row["render_detect_ms"])))
    if not observations:
        raise ValueError(f"no observations found in {path}")
    return observations


def _render_offline_failure_montage(observations, threshold, target_label, path):
    """Render threshold-specific failure cards when raw RGB was not persisted."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    grouped = defaultdict(list)
    for row in observations:
        if row.case_class == "INTERIOR":
            grouped[row.case_id].append(row)
    misses = [(case_id, rows) for case_id, rows in grouped.items()
              if _candidate_for_case(rows, threshold, target_label) is None][:12]
    fig, axes = plt.subplots(3, 4, figsize=(10, 7), constrained_layout=True)
    for axis, (case_id, rows) in zip(axes.flat, misses):
        actual = rows[0].actual_center_base
        confidences = [row.confidence for row in rows
                       if row.predicted_label == target_label
                       and row.confidence is not None]
        max_conf = max(confidences) if confidences else None
        axis.text(0.5, 0.62, case_id, ha="center", va="center",
                  fontsize=10, weight="bold")
        axis.text(0.5, 0.42,
                  f"actual xy=({actual[0]:.3f}, {actual[1]:.3f})\n"
                  f"max bottle conf={max_conf:.3f}" if max_conf is not None
                  else f"actual xy=({actual[0]:.3f}, {actual[1]:.3f})\n"
                       "no bottle candidate",
                  ha="center", va="center", fontsize=9)
        axis.set_facecolor("#f8d7da")
        axis.set_xticks([])
        axis.set_yticks([])
    for axis in list(axes.flat)[len(misses):]:
        axis.axis("off")
    fig.suptitle(
        f"Offline failure cases at confidence >= {threshold:g}\n"
        "Diagnostic cards only: raw RGB was not persisted", fontsize=12)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def reevaluate_calibration(
        observations_csv: pathlib.Path, output_dir: pathlib.Path,
        thresholds: Sequence[float] = (0.05, 0.075, 0.10, 0.15, 0.20, 0.25, 0.30)):
    """Re-evaluate saved raw candidates without rendering or YOLO inference."""
    observations_csv = pathlib.Path(observations_csv)
    output_dir = pathlib.Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    observations = _read_observations_csv(observations_csv)
    cfg = load_config()
    domain_x, domain_y = _domain_ranges(cfg)
    target_label = "bottle"
    ordered_thresholds = tuple(sorted({float(value) for value in thresholds}))
    if not ordered_thresholds:
        raise ValueError("thresholds must not be empty")
    metrics = [evaluate_threshold(
        observations, threshold, target_label, domain_x, domain_y)
        for threshold in ordered_thresholds]
    selected = max((row.threshold for row in metrics if row.gate_passed),
                   default=None)
    diagnostic_threshold = selected if selected is not None else max(ordered_thresholds)
    source_summary_path = observations_csv.with_name("perception_summary.json")
    source_summary = None
    if source_summary_path.exists():
        source_summary = json.loads(source_summary_path.read_text(encoding="utf-8"))
    summary = {
        "suite": "perception_calibration_offline_reevaluation",
        "source_observations": str(observations_csv.resolve()),
        "source_summary": str(source_summary_path.resolve())
        if source_summary is not None else None,
        "production_threshold_before": cfg["detector"]["confidence_threshold"],
        "threshold_metrics": [asdict(row) for row in metrics],
        "gate": {
            "passed": selected is not None,
            "selected_threshold": selected,
            "selection_rule": "highest threshold satisfying all requirements",
            "requirements": {"recall_pct_min": 95.0,
                             "false_positive_rate_pct_max": 2.0,
                             "xy_error_p95_mm_max": 10.0},
        },
        "boundary_diagnostic": _boundary_metrics(
            observations, diagnostic_threshold, target_label, domain_x, domain_y),
        "offline_only": True,
        "inference_rerun": False,
        "production_config_modified": False,
    }
    _write_dataclass_csv(output_dir / "perception_threshold_sweep.csv", metrics)
    (output_dir / "perception_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    _render_threshold_plot(metrics, output_dir / "confidence_recall_fpr.png")
    _render_offline_failure_montage(
        observations, diagnostic_threshold, target_label,
        output_dir / "failure_montage_offline.png")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary


def run_perception_calibration(spec: PerceptionBenchmarkSpec, output_dir: pathlib.Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    domain_x, domain_y = _domain_ranges(cfg)
    started = time.perf_counter()
    observations, thumbnails = _collect_observations(spec, domain_x, domain_y)
    metrics = [evaluate_threshold(
        observations, threshold, spec.target_label, domain_x, domain_y)
        for threshold in spec.confidence_thresholds]
    passing = [row for row in metrics if row.gate_passed]
    selected = max((row.threshold for row in passing), default=None)
    boundary_threshold = selected if selected is not None else max(spec.confidence_thresholds)
    summary = {
        "suite": "perception_calibration", "spec": asdict(spec),
        "production_threshold_before": cfg["detector"]["confidence_threshold"],
        "threshold_metrics": [asdict(row) for row in metrics],
        "gate": {
            "passed": selected is not None,
            "selected_threshold": selected,
            "requirements": {"recall_pct_min": 95.0,
                             "false_positive_rate_pct_max": 2.0,
                             "xy_error_p95_mm_max": 10.0},
        },
        "boundary_diagnostic": _boundary_metrics(
            observations, boundary_threshold, spec.target_label, domain_x, domain_y),
        "wall_clock_s": round(time.perf_counter() - started, 2),
        "production_config_modified": False,
    }
    _write_dataclass_csv(output_dir / "perception_observations.csv", observations)
    _write_dataclass_csv(output_dir / "perception_threshold_sweep.csv", metrics)
    (output_dir / "perception_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    _render_threshold_plot(metrics, output_dir / "confidence_recall_fpr.png")
    _render_failure_montage(
        observations, thumbnails, boundary_threshold, spec.target_label,
        output_dir / "failure_montage.png")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary


def _render_e2e_heatmap(backend, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cells = {(row["grid_x_m"], row["grid_y_m"]): row for row in backend["cells"]}
    values = np.array([[cells[(x, y)]["success_rate_pct"] for x in DEFAULT_X_VALUES]
                       for y in E2E_INTERIOR_Y])
    fig, axis = plt.subplots(figsize=(7, 5), constrained_layout=True)
    image = axis.imshow(values, origin="lower", cmap="RdYlGn", vmin=0, vmax=100,
                        aspect="auto")
    axis.set_xticks(range(5), [f"{x:.3f}" for x in DEFAULT_X_VALUES])
    axis.set_yticks(range(4), [f"{y:.3f}" for y in E2E_INTERIOR_Y])
    axis.set_xlabel("Base-frame x (m)")
    axis.set_ylabel("Base-frame y (m)")
    axis.set_title("YOLO/RGB-D interior end-to-end")
    for yi, y in enumerate(E2E_INTERIOR_Y):
        for xi, x in enumerate(DEFAULT_X_VALUES):
            cell = cells[(x, y)]
            axis.text(xi, yi, f"{cell['successes']}/{cell['trials']}",
                      ha="center", va="center",
                      color="black" if 30 < values[yi, xi] < 75 else "white")
    fig.colorbar(image, ax=axis, label="Success rate (%)")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def run_perception_e2e(output_dir: pathlib.Path, calibration_summary: pathlib.Path,
                       model="piper_real", seed=20260713):
    calibration = json.loads(calibration_summary.read_text(encoding="utf-8"))
    if not calibration["gate"]["passed"]:
        raise RuntimeError("perception calibration Gate did not pass")
    selected = float(calibration["gate"]["selected_threshold"])
    cfg = load_config()
    configured = float(cfg["detector"]["confidence_threshold"])
    if not np.isclose(configured, selected):
        raise RuntimeError(
            f"configured detector threshold {configured} does not match calibrated {selected}")
    domain_x, domain_y = _domain_ranges(cfg)
    spec = GraspGridSpec(
        target_label="bottle", x_values_m=DEFAULT_X_VALUES,
        y_values_m=E2E_INTERIOR_Y, repeats_per_cell=10, xy_jitter_m=0.003,
        yaw_range_deg=(0.0, 360.0), seed=seed, model=model, backends=("yolo",))
    cases = generate_grasp_grid_cases(spec, domain_x, domain_y)
    started = time.perf_counter()
    records = run_grasp_grid("yolo", spec, cases, "radial_single")
    backend = aggregate_grasp_heatmap(records)["yolo"]
    summary = {
        "suite": "perception_e2e", "calibrated_threshold": selected,
        "sim_gt_reference": {"trials": 250, "successes": 250,
                             "success_rate_pct": 100.0},
        "yolo": backend, "passed": backend["target_met"],
        "wall_clock_s": round(time.perf_counter() - started, 2),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_dataclass_csv(output_dir / "perception_e2e_trials.csv", records)
    (output_dir / "perception_e2e_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    _render_e2e_heatmap(backend, output_dir / "perception_e2e_heatmap.png")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary
