"""Repeatable bottle pick-success heatmap benchmark for MuJoCo.

The in-domain benchmark measures manipulation with sim_gt first, then runs the
same immutable cases through YOLO/RGB-D only when the manipulation baseline
passes. Positions outside the configured operating domain are diagnostics and
are never included in the grasp-success denominator.
"""
from __future__ import annotations

import csv
import json
import math
import os
import pathlib
import statistics
import time
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from typing import Literal, Sequence

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

from .config import load_config
from .executor import TaskExecutor
from .world import SimWorld
from sim.constrained_ik import clear_constrained_ik_cache


ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT = pathlib.Path(__file__).resolve().parent / "benchmark_results" / "bottle_heatmap"
DEFAULT_X_VALUES = (0.34, 0.36, 0.38, 0.40, 0.42)
DEFAULT_Y_VALUES = (-0.16, -0.115, -0.07, -0.025, 0.02)
DEFAULT_OOD_Y_VALUES = (0.05, 0.12)
REPRESENTATIVE_GATE_POINTS = (
    (0.34, -0.16), (0.34, -0.025), (0.36, -0.115), (0.36, 0.02),
    (0.38, -0.025), (0.40, -0.115), (0.42, -0.115), (0.42, 0.02))
DOMAIN_IN = "IN_DOMAIN"
DOMAIN_OUT = "OUT_OF_DOMAIN_DIAGNOSTIC"


@dataclass(frozen=True)
class GraspGridSpec:
    target_label: str = "bottle"
    x_values_m: tuple[float, ...] = DEFAULT_X_VALUES
    y_values_m: tuple[float, ...] = DEFAULT_Y_VALUES
    repeats_per_cell: int = 10
    xy_jitter_m: float = 0.003
    yaw_range_deg: tuple[float, float] = (0.0, 360.0)
    seed: int = 20260713
    model: str = "piper_real"
    backends: tuple[str, ...] = ("sim_gt", "yolo")


@dataclass(frozen=True)
class GraspTrialCase:
    case_id: str
    grid_x_m: float
    grid_y_m: float
    actual_x_m: float
    actual_y_m: float
    yaw_deg: float
    repeat_index: int
    domain_class: Literal["IN_DOMAIN", "OUT_OF_DOMAIN_DIAGNOSTIC"]


@dataclass
class GraspTrialRecord:
    case_id: str
    backend: str
    target_label: str
    grid_x_m: float
    grid_y_m: float
    actual_x_m: float
    actual_y_m: float
    yaw_deg: float
    repeat_index: int
    domain_class: str
    success: bool
    initial_failed_state: str
    initial_failure_reason: str
    failed_state: str
    failure_reason: str
    lift_height_mm: float | None
    task_duration_ms: float | None
    detect_source_ms: float | None
    execute_pick_ms: float | None
    source_center_error_mm: float | None
    selected_direction_deg: float | None
    direction_policy_mode: str | None
    direction_policy_band: str | None
    direction_reference_deg: float | None
    planned_grasp_z_m: float | None
    grasp_z_clearance_m: float | None
    max_cartesian_deviation_mm: float | None
    lift_horizontal_path_drift_mm: float | None
    execution_trace_json: str
    target_settle_displacement_mm: float
    unexpected_contacts_json: str
    state_log_json: str


@dataclass(frozen=True)
class PickSceneIsolationReport:
    target_body: str
    hidden_bodies: tuple[str, ...]
    disabled_geom_ids: tuple[int, ...]
    target_settle_displacement_mm: float
    unexpected_contacts: tuple[str, ...]


@dataclass(frozen=True)
class DirectionProbeResult:
    grid_x_m: float
    grid_y_m: float
    direction_deg: float
    ik_feasible: bool
    collision_free: bool
    success: bool
    lift_height_mm: float | None
    failure_reason: str


def _domain_ranges(cfg):
    domain = cfg.get("grasp_planner", {}).get("operating_domain", {})
    return tuple(domain.get("x", (-math.inf, math.inf))), tuple(
        domain.get("y", (-math.inf, math.inf)))


def _clip_inside_domain(value, bounds):
    # MuJoCo settling can add a few ulps at an exact ODD boundary. Keep the
    # measured pose one micrometre inside while preserving the nominal cell.
    epsilon = min(1e-6, max(0.0, (bounds[1] - bounds[0]) / 4.0))
    return float(np.clip(value, bounds[0] + epsilon, bounds[1] - epsilon))


def generate_grasp_grid_cases(
        spec: GraspGridSpec,
        domain_x: tuple[float, float],
        domain_y: tuple[float, float]) -> list[GraspTrialCase]:
    """Generate deterministic local perturbations, clipped to the declared ODD."""
    if spec.repeats_per_cell < 1:
        raise ValueError("repeats_per_cell must be at least one")
    if spec.xy_jitter_m < 0:
        raise ValueError("xy_jitter_m must be non-negative")
    rng = np.random.default_rng(spec.seed)
    cases = []
    for xi, x in enumerate(spec.x_values_m):
        for yi, y in enumerate(spec.y_values_m):
            if not (domain_x[0] <= x <= domain_x[1] and domain_y[0] <= y <= domain_y[1]):
                raise ValueError(f"nominal grid point {(x, y)} is outside operating domain")
            for repeat in range(spec.repeats_per_cell):
                jitter = rng.uniform(-spec.xy_jitter_m, spec.xy_jitter_m, size=2)
                actual_x = _clip_inside_domain(x + jitter[0], domain_x)
                actual_y = _clip_inside_domain(y + jitter[1], domain_y)
                yaw = float(rng.uniform(*spec.yaw_range_deg))
                cases.append(GraspTrialCase(
                    case_id=f"x{xi + 1:02d}-y{yi + 1:02d}-r{repeat + 1:02d}",
                    grid_x_m=float(x), grid_y_m=float(y),
                    actual_x_m=actual_x, actual_y_m=actual_y,
                    yaw_deg=yaw, repeat_index=repeat + 1,
                    domain_class=DOMAIN_IN))
    return cases


def generate_ood_diagnostic_cases(
        x_values: Sequence[float], y_values: Sequence[float]) -> list[GraspTrialCase]:
    cases = []
    for xi, x in enumerate(x_values):
        for yi, y in enumerate(y_values):
            cases.append(GraspTrialCase(
                case_id=f"ood-x{xi + 1:02d}-y{yi + 1:02d}",
                grid_x_m=float(x), grid_y_m=float(y),
                actual_x_m=float(x), actual_y_m=float(y), yaw_deg=0.0,
                repeat_index=1, domain_class=DOMAIN_OUT))
    return cases


def _freejoint_qpos_address(model, body_name):
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body < 0 or model.body_jntnum[body] < 1:
        raise ValueError(f"body {body_name!r} has no free joint")
    joint = model.body_jntadr[body]
    return int(model.jnt_qposadr[joint])


def _target_body_name(cfg, target_label):
    return cfg.get("objects", {}).get("body_of", {}).get(target_label, target_label)


def _move_body(world, body_name, xyz, yaw_deg=0.0):
    address = _freejoint_qpos_address(world.model, body_name)
    world.data.qpos[address:address + 3] = xyz
    half = np.deg2rad(yaw_deg) / 2.0
    world.data.qpos[address + 3:address + 7] = [np.cos(half), 0.0, 0.0, np.sin(half)]


def _body_geom_ids(model, body_name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        return ()
    return tuple(int(gid) for gid in np.flatnonzero(model.geom_bodyid == body_id))


def isolate_pick_scene(world: SimWorld, target_label: str,
                       cfg: dict) -> PickSceneIsolationReport:
    """Remove non-target task objects from one disposable benchmark model."""
    target_body = _target_body_name(cfg, target_label)
    requested_xy = np.asarray(world.body_pos(target_body)[:2], float).copy()
    hidden = tuple(name for name in (
        "bottle", "cup", "ycb_bottle", "bowl", "demo_bottle", "demo_box")
                   if name != target_body and
                   mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0)
    disabled = []
    for body_name in hidden:
        for geom_id in _body_geom_ids(world.model, body_name):
            world.model.geom_contype[geom_id] = 0
            world.model.geom_conaffinity[geom_id] = 0
            world.model.geom_matid[geom_id] = -1
            world.model.geom_rgba[geom_id] = [0.0, 0.0, 0.0, 0.0]
            disabled.append(geom_id)

    mujoco.mj_forward(world.model, world.data)
    world.settle(700)
    settled_xy = np.asarray(world.body_pos(target_body)[:2], float)
    displacement_mm = float(np.linalg.norm(settled_xy - requested_xy) * 1000.0)

    target_id = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, target_body)
    hidden_ids = {
        mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, name): name
        for name in hidden}
    contacts = []
    for contact in world.data.contact:
        bodies = (int(world.model.geom_bodyid[contact.geom1]),
                  int(world.model.geom_bodyid[contact.geom2]))
        if target_id not in bodies:
            continue
        other = bodies[1] if bodies[0] == target_id else bodies[0]
        if other in hidden_ids:
            contacts.append(hidden_ids[other])
    return PickSceneIsolationReport(
        target_body=target_body, hidden_bodies=hidden,
        disabled_geom_ids=tuple(sorted(disabled)),
        target_settle_displacement_mm=round(displacement_mm, 3),
        unexpected_contacts=tuple(sorted(set(contacts))))


def reset_target_for_trial(world: SimWorld, cfg, target_label: str,
                           case: GraspTrialCase) -> PickSceneIsolationReport:
    """Reset the arm and isolate one target object before a measured pick."""
    mujoco.mj_resetData(world.model, world.data)
    home = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_KEY, "home")
    world.ctrl[:] = world.model.key_ctrl[home]
    world.ready_ctrl = np.array(world.model.key_ctrl[home])
    world.data.ctrl[:] = world.ctrl
    world.frames.clear()

    target_body = _target_body_name(cfg, target_label)
    for body_name, hidden_xyz in (
            ("bottle", (2.0, 2.0, 0.20)),
            ("cup", (2.0, -2.0, 0.20)),
            ("ycb_bottle", (2.0, 2.0, 0.20))):
        if body_name == target_body:
            continue
        body_id = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id >= 0 and world.model.body_jntnum[body_id] > 0:
            _move_body(world, body_name, hidden_xyz)

    _move_body(world, target_body,
               (case.actual_x_m, case.actual_y_m, 0.20), case.yaw_deg)
    mujoco.mj_forward(world.model, world.data)
    report = isolate_pick_scene(world, target_label, cfg)
    clear_constrained_ik_cache()
    return report


def _state_duration(summary, state):
    return round(sum(s.get("duration_ms", 0.0) for s in summary["states"]
                     if s["state"] == state), 1)


def _failed_state(summary):
    if summary["success"]:
        return ""
    return next((s["state"] for s in reversed(summary["states"])
                 if not s["success"]), "")


def _failure_reason(summary):
    if summary["success"]:
        return ""
    return next((s.get("reason", "STATE_FAILED") for s in reversed(summary["states"])
                 if not s["success"]), "STATE_FAILED")


def _initial_failed_state(summary):
    if summary["success"]:
        return ""
    return next((s["state"] for s in summary["states"] if not s["success"]), "")


def _initial_failure_reason(summary):
    if summary["success"]:
        return ""
    return next((s.get("reason", "STATE_FAILED") for s in summary["states"]
                 if not s["success"]), "STATE_FAILED")


def _configured_world(backend, model_name, strategy):
    cfg = load_config()
    cfg["detector"]["backend"] = backend
    cfg["grasp"]["strategy"] = strategy
    canonical = "piper_real" if model_name == "real" else model_name
    config_key = "real" if canonical == "piper_real" else canonical
    selected = cfg.get("model", {}).get(config_key, {})
    if selected.get("scan_poses"):
        cfg["detector"]["scan_poses"] = selected["scan_poses"]
    for section in ("grasp", "place", "scene", "detector"):
        if isinstance(selected.get(section), dict):
            cfg[section].update(selected[section])
    scene_path = ROOT / selected["scene"] if selected.get("scene") else None
    world = SimWorld.load(scene=scene_path, record=False,
                          use_analytic_ik=bool(selected.get("analytic_ik", False)))
    return cfg, world, canonical


def run_grasp_grid(backend: str, spec: GraspGridSpec,
                   cases: Sequence[GraspTrialCase], strategy="radial_single"):
    cfg, world, canonical = _configured_world(backend, spec.model, strategy)
    records = []
    total = len(cases)
    for index, case in enumerate(cases, start=1):
        isolation = reset_target_for_trial(world, cfg, spec.target_label, case)
        summary = TaskExecutor(world, cfg, verbose=False).pick(spec.target_label)
        data = summary["data"]
        record = GraspTrialRecord(
            case_id=case.case_id, backend=backend, target_label=spec.target_label,
            grid_x_m=case.grid_x_m, grid_y_m=case.grid_y_m,
            actual_x_m=case.actual_x_m, actual_y_m=case.actual_y_m,
            yaw_deg=case.yaw_deg, repeat_index=case.repeat_index,
            domain_class=case.domain_class, success=bool(summary["success"]),
            initial_failed_state=_initial_failed_state(summary),
            initial_failure_reason=_initial_failure_reason(summary),
            failed_state=_failed_state(summary), failure_reason=_failure_reason(summary),
            lift_height_mm=data.get("rose_mm"), task_duration_ms=data.get("task_duration_ms"),
            detect_source_ms=_state_duration(summary, "DETECT_SOURCE"),
            execute_pick_ms=_state_duration(summary, "EXECUTE_PICK"),
            source_center_error_mm=data.get("source_center_error_mm"),
            selected_direction_deg=data.get("selected_direction_deg"),
            direction_policy_mode=data.get("direction_policy_mode"),
            direction_policy_band=data.get("direction_policy_band"),
            direction_reference_deg=data.get("direction_reference_deg"),
            planned_grasp_z_m=data.get("planned_grasp_z_m"),
            grasp_z_clearance_m=data.get("grasp_z_clearance_m"),
            max_cartesian_deviation_mm=data.get("max_cartesian_deviation_mm"),
            lift_horizontal_path_drift_mm=data.get("lift_horizontal_path_drift_mm"),
            execution_trace_json=json.dumps(
                data.get("execution_trace"), ensure_ascii=False),
            target_settle_displacement_mm=isolation.target_settle_displacement_mm,
            unexpected_contacts_json=json.dumps(isolation.unexpected_contacts),
            state_log_json=json.dumps(summary["states"], ensure_ascii=False))
        records.append(record)
        print(f"[{canonical}/{backend}/{spec.target_label}] {index:03d}/{total}: "
              f"{'PASS' if record.success else 'FAIL'} "
              f"xy=({case.actual_x_m:.3f},{case.actual_y_m:.3f}) "
              f"reason={record.failure_reason or 'OK'}")
    return records


def _representative_failed_cells(records: Sequence[GraspTrialRecord], max_cells=8):
    points = sorted({(record.grid_x_m, record.grid_y_m) for record in records
                     if not record.success and record.domain_class == DOMAIN_IN})
    if len(points) <= max_cells:
        return points
    indices = np.linspace(0, len(points) - 1, max_cells).round().astype(int)
    return [points[index] for index in dict.fromkeys(indices)]


def run_direction_probe(
        failed_cells: Sequence[tuple[float, float]],
        directions_deg: Sequence[float] = (0, 45, 90, 135, 180, 225, 270, 315),
        max_cells: int = 8, retries: int = 0,
        target_label="bottle", model_name="piper_real") -> list[DirectionProbeResult]:
    """Force one constrained candidate at a time; never changes production config."""
    cfg, world, canonical = _configured_world(
        "sim_gt", model_name, "multi_candidate_v1_1")
    cfg["grasp"]["max_retries"] = int(retries)
    cfg["grasp_planner"]["candidate_count"] = 1
    domain_x, domain_y = _domain_ranges(cfg)
    selected_cells = _representative_failed_cells(
        [GraspTrialRecord(
            case_id="", backend="sim_gt", target_label=target_label,
            grid_x_m=x, grid_y_m=y, actual_x_m=x, actual_y_m=y,
            yaw_deg=0.0, repeat_index=1, domain_class=DOMAIN_IN,
            success=False, initial_failed_state="", initial_failure_reason="",
            failed_state="", failure_reason="", lift_height_mm=None,
            task_duration_ms=None, detect_source_ms=None, execute_pick_ms=None,
            source_center_error_mm=None, target_settle_displacement_mm=0.0,
            selected_direction_deg=None, max_cartesian_deviation_mm=None,
            direction_policy_mode=None, direction_policy_band=None,
            direction_reference_deg=None,
            planned_grasp_z_m=None, grasp_z_clearance_m=None,
            lift_horizontal_path_drift_mm=None, execution_trace_json="null",
            unexpected_contacts_json="[]", state_log_json="[]")
         for x, y in failed_cells], max_cells)
    results = []
    total = len(selected_cells) * len(directions_deg)
    index = 0
    for x, y in selected_cells:
        for direction in directions_deg:
            index += 1
            case = GraspTrialCase(
                case_id=f"probe-{x:.3f}-{y:.3f}-{float(direction):.0f}",
                grid_x_m=x, grid_y_m=y,
                actual_x_m=_clip_inside_domain(x, domain_x),
                actual_y_m=_clip_inside_domain(y, domain_y),
                yaw_deg=0.0, repeat_index=1, domain_class=DOMAIN_IN)
            reset_target_for_trial(world, cfg, target_label, case)
            settled = world.body_pos(_target_body_name(cfg, target_label))
            radial_deg = float(np.rad2deg(np.arctan2(settled[1], settled[0])))
            cfg["grasp_planner"]["start_angle_offset_deg"] = float(direction) - radial_deg
            profile_cfg = cfg.setdefault("grasp", {}).setdefault(
                "profiles", {}).setdefault(target_label, {})
            profile_cfg["strategy"] = "multi_candidate_v1_1"
            profile_cfg["preferred_directions_deg"] = [float(direction)]
            summary = TaskExecutor(world, cfg, verbose=False).pick(target_label)
            data = summary["data"]
            reports = data.get("candidate_reports", [])
            report = reports[0] if reports else {}
            rejections = set(report.get("rejection_reasons", []))
            ik_feasible = bool(report) and "IK_BOUNDED_ORIENTATION_UNREACHABLE" not in rejections
            collision_free = bool(report) and "HARD_COLLISION_STATIC_OR_SELF" not in rejections
            result = DirectionProbeResult(
                grid_x_m=x, grid_y_m=y, direction_deg=float(direction),
                ik_feasible=ik_feasible, collision_free=collision_free,
                success=bool(summary["success"]), lift_height_mm=data.get("rose_mm"),
                failure_reason=_failure_reason(summary))
            results.append(result)
            print(f"[{canonical}/direction-probe] {index:03d}/{total}: "
                  f"xy=({x:.3f},{y:.3f}) dir={direction:.0f} "
                  f"{'PASS' if result.success else 'FAIL'} "
                  f"reason={result.failure_reason or 'OK'}")
    return results


def summarize_direction_probe(results: Sequence[DirectionProbeResult]):
    grouped = defaultdict(list)
    for result in results:
        grouped[(result.grid_x_m, result.grid_y_m)].append(result)
    cells = []
    for (x, y), rows in sorted(grouped.items()):
        if any(row.success for row in rows):
            classification = "DIRECTION_SELECTION"
        elif any(row.ik_feasible and row.collision_free for row in rows):
            classification = "EXECUTION_OR_CONTACT"
        else:
            classification = "REACHABILITY_OR_COLLISION"
        cells.append({
            "grid_x_m": x, "grid_y_m": y, "classification": classification,
            "successful_directions_deg": [row.direction_deg for row in rows if row.success],
            "ik_feasible_directions_deg": [row.direction_deg for row in rows if row.ik_feasible],
        })
    return {"trials": len(results), "cells": cells,
            "classification_counts": dict(Counter(
                cell["classification"] for cell in cells))}


def _mean(values):
    values = [float(value) for value in values if value is not None]
    return round(statistics.fmean(values), 2) if values else None


def _percentile(values, percentile):
    values = [float(value) for value in values if value is not None]
    return round(float(np.percentile(values, percentile)), 2) if values else None


def _wilson_interval(successes, trials, z=1.959963984540054):
    if trials == 0:
        return None
    p = successes / trials
    denominator = 1 + z * z / trials
    center = (p + z * z / (2 * trials)) / denominator
    margin = z * math.sqrt(p * (1 - p) / trials + z * z / (4 * trials * trials)) / denominator
    return [round(100 * max(0.0, center - margin), 2),
            round(100 * min(1.0, center + margin), 2)]


def summarize_backend(records: Sequence[GraspTrialRecord]):
    in_domain = [record for record in records if record.domain_class == DOMAIN_IN]
    successes = sum(record.success for record in in_domain)
    cells = defaultdict(list)
    for record in in_domain:
        cells[(record.grid_x_m, record.grid_y_m)].append(record)
    cell_rows = []
    for (x, y), rows in sorted(cells.items()):
        cell_successes = sum(row.success for row in rows)
        cell_rows.append({
            "grid_x_m": x, "grid_y_m": y, "trials": len(rows),
            "successes": cell_successes,
            "success_rate_pct": round(100 * cell_successes / len(rows), 2),
            "wilson_95_pct": _wilson_interval(cell_successes, len(rows)),
        })
    return {
        "trials": len(in_domain), "successes": successes,
        "success_rate_pct": round(100 * successes / len(in_domain), 2) if in_domain else None,
        "wilson_95_pct": _wilson_interval(successes, len(in_domain)),
        "target_met": bool(in_domain and successes / len(in_domain) >= 0.85),
        "mean_task_duration_ms": _mean(row.task_duration_ms for row in in_domain),
        "p95_task_duration_ms": _percentile((row.task_duration_ms for row in in_domain), 95),
        "mean_lift_height_mm": _mean(row.lift_height_mm for row in in_domain),
        "mean_source_center_error_mm": _mean(
            row.source_center_error_mm for row in in_domain),
        "max_target_settle_displacement_mm": max(
            (row.target_settle_displacement_mm for row in in_domain), default=None),
        "unexpected_contact_trials": sum(
            bool(json.loads(row.unexpected_contacts_json)) for row in in_domain),
        "failure_state_counts": dict(Counter(
            row.failed_state for row in in_domain if not row.success)),
        "failure_reason_counts": dict(Counter(
            row.failure_reason for row in in_domain if not row.success)),
        "initial_failure_state_counts": dict(Counter(
            row.initial_failed_state for row in in_domain if not row.success)),
        "initial_failure_reason_counts": dict(Counter(
            row.initial_failure_reason for row in in_domain if not row.success)),
        "cells": cell_rows,
    }


def aggregate_grasp_heatmap(records: Sequence[GraspTrialRecord]):
    backends = sorted({record.backend for record in records})
    return {backend: summarize_backend(
        [record for record in records if record.backend == backend]) for backend in backends}


def _write_csv(path, records):
    if not records:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(records[0])))
        writer.writeheader()
        writer.writerows(asdict(record) for record in records)


def _write_dataclass_csv(path, records):
    if not records:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(records[0])))
        writer.writeheader()
        writer.writerows(asdict(record) for record in records)


def render_grasp_heatmap(summary, output_path, x_values, y_values, ood_y_values):
    """Render sim_gt, YOLO, and YOLO-minus-sim_gt as one publication-ready PNG."""
    os.environ.setdefault("MPLCONFIGDIR", str(output_path.parent / ".matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    all_y = tuple(y_values) + tuple(y for y in ood_y_values if y not in y_values)
    backend_cells = {}
    sim_title = "sim_gt manipulation"
    for backend in ("sim_gt", "yolo"):
        backend_cells[backend] = {
            (cell["grid_x_m"], cell["grid_y_m"]): cell
            for cell in summary.get("backends", {}).get(backend, {}).get("cells", [])}
    if not backend_cells["sim_gt"] and summary.get("smoke_gate", {}).get("cells"):
        backend_cells["sim_gt"] = {
            (cell["grid_x_m"], cell["grid_y_m"]): cell
            for cell in summary["smoke_gate"]["cells"]}
        sim_title = "sim_gt smoke gate (1 trial/cell)"

    def matrix(backend):
        values = np.full((len(all_y), len(x_values)), np.nan)
        labels = np.full(values.shape, "N/A", dtype=object)
        for yi, y in enumerate(all_y):
            for xi, x in enumerate(x_values):
                cell = backend_cells[backend].get((x, y))
                if cell:
                    values[yi, xi] = cell["success_rate_pct"]
                    labels[yi, xi] = f"{cell['successes']}/{cell['trials']}"
        return values, labels

    sim_values, sim_labels = matrix("sim_gt")
    yolo_values, yolo_labels = matrix("yolo")
    delta = yolo_values - sim_values
    delta_labels = np.full(delta.shape, "N/A", dtype=object)
    for index in np.ndindex(delta.shape):
        if np.isfinite(delta[index]):
            delta_labels[index] = f"{delta[index]:+.0f}"

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.6), sharex=True, sharey=True,
                             constrained_layout=True)
    panels = (
        (sim_values, sim_labels, sim_title, "RdYlGn", 0, 100, "%"),
        (yolo_values, yolo_labels, "YOLO/RGB-D end-to-end", "RdYlGn", 0, 100, "%"),
        (delta, delta_labels, "YOLO - sim_gt", "RdYlGn", -100, 100, "percentage points"),
    )
    for axis, (values, labels, title, cmap, low, high, color_label) in zip(axes, panels):
        masked = np.ma.masked_invalid(values)
        image = axis.imshow(masked, origin="lower", cmap=cmap, vmin=low, vmax=high,
                            aspect="auto")
        axis.set_title(title)
        axis.set_xlabel("Base-frame x (m)")
        axis.set_xticks(range(len(x_values)), [f"{x:.3f}" for x in x_values])
        axis.set_yticks(range(len(all_y)), [f"{y:.3f}" for y in all_y])
        for yi, y in enumerate(all_y):
            for xi, _ in enumerate(x_values):
                text = labels[yi, xi]
                color = "black" if not np.isfinite(values[yi, xi]) or 30 < values[yi, xi] < 75 else "white"
                axis.text(xi, yi, text, ha="center", va="center", fontsize=8, color=color)
        fig.colorbar(image, ax=axis, shrink=0.72, label=color_label)
    axes[0].set_ylabel("Base-frame y (m); OOD rows are N/A")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def run_grasp_heatmap_suite(spec: GraspGridSpec, output_dir: pathlib.Path,
                            strategy="radial_single", ood_y_values=DEFAULT_OOD_Y_VALUES):
    output_dir.mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    domain_x, domain_y = _domain_ranges(cfg)
    smoke_spec = GraspGridSpec(
        target_label=spec.target_label, x_values_m=spec.x_values_m,
        y_values_m=spec.y_values_m, repeats_per_cell=1, xy_jitter_m=0.0,
        yaw_range_deg=(0.0, 0.0), seed=spec.seed, model=spec.model,
        backends=("sim_gt",))
    smoke_cases = generate_grasp_grid_cases(smoke_spec, domain_x, domain_y)
    representative_cases = [GraspTrialCase(
        case_id=f"representative-{index + 1:02d}",
        grid_x_m=x, grid_y_m=y,
        actual_x_m=_clip_inside_domain(x, domain_x),
        actual_y_m=_clip_inside_domain(y, domain_y),
        yaw_deg=0.0, repeat_index=1, domain_class=DOMAIN_IN)
        for index, (x, y) in enumerate(REPRESENTATIVE_GATE_POINTS)]
    formal_cases = generate_grasp_grid_cases(spec, domain_x, domain_y)
    ood_cases = generate_ood_diagnostic_cases(spec.x_values_m, ood_y_values)
    cases_payload = {
        "spec": asdict(spec), "operating_domain": {"x": domain_x, "y": domain_y},
        "in_domain_cases": [asdict(case) for case in formal_cases],
        "out_of_domain_diagnostics": [asdict(case) for case in ood_cases],
    }
    (output_dir / "cases.json").write_text(
        json.dumps(cases_payload, ensure_ascii=False, indent=2), encoding="utf-8")

    started = time.perf_counter()
    representative_records = run_grasp_grid(
        "sim_gt", smoke_spec, representative_cases, strategy)
    representative_summary = summarize_backend(representative_records)
    representative_gate = {
        "required_successes": 7, "trials": len(representative_records),
        "successes": sum(record.success for record in representative_records),
        "passed": sum(record.success for record in representative_records) >= 7,
        "cells": representative_summary["cells"],
        "failure_reason_counts": representative_summary["failure_reason_counts"],
        "initial_failure_reason_counts":
            representative_summary["initial_failure_reason_counts"],
    }
    smoke_records = (run_grasp_grid("sim_gt", smoke_spec, smoke_cases, strategy)
                     if representative_gate["passed"] else [])
    smoke_successes = sum(record.success for record in smoke_records)
    smoke_summary = summarize_backend(smoke_records)
    diagnostics = run_grasp_grid("sim_gt", spec, ood_cases, strategy)
    records = []
    measurement_valid = bool(smoke_records and
        smoke_summary["max_target_settle_displacement_mm"] is not None and
        smoke_summary["max_target_settle_displacement_mm"] <= 5.0 and
        smoke_summary["unexpected_contact_trials"] == 0 and
        smoke_summary["initial_failure_reason_counts"].get("OBJECT_OUT_OF_ODD", 0) == 0)
    functional_passed = bool(smoke_records and smoke_successes >= 23)
    gate = {"required_successes": 23, "trials": len(smoke_records),
            "successes": smoke_successes, "executed": bool(smoke_records),
            "measurement_valid": measurement_valid,
            "functional_passed": functional_passed,
            "passed": measurement_valid and functional_passed,
            "max_target_settle_displacement_mm":
                smoke_summary["max_target_settle_displacement_mm"],
            "unexpected_contact_trials": smoke_summary["unexpected_contact_trials"],
            "cells": smoke_summary["cells"],
            "failure_state_counts": smoke_summary["failure_state_counts"],
            "failure_reason_counts": smoke_summary["failure_reason_counts"],
            "initial_failure_state_counts": smoke_summary["initial_failure_state_counts"],
            "initial_failure_reason_counts": smoke_summary["initial_failure_reason_counts"]}
    direction_probe = []
    if gate["passed"]:
        sim_records = run_grasp_grid("sim_gt", spec, formal_cases, strategy)
        records.extend(sim_records)
        sim_summary = summarize_backend(sim_records)
        if sim_summary["target_met"] and "yolo" in spec.backends:
            records.extend(run_grasp_grid("yolo", spec, formal_cases, strategy))
    elif representative_gate["passed"] and measurement_valid:
        failed_cells = [(record.grid_x_m, record.grid_y_m)
                        for record in smoke_records if not record.success]
        direction_probe = run_direction_probe(
            failed_cells, max_cells=8, retries=0,
            target_label=spec.target_label, model_name=spec.model)

    backend_summary = aggregate_grasp_heatmap(records)
    diagnostic_rows = [asdict(record) for record in diagnostics]
    summary = {
        "suite": "grasp_heatmap", "target_label": spec.target_label,
        "model": spec.model, "strategy": strategy, "seed": spec.seed,
        "success_threshold_pct": 85.0, "smoke_gate": gate,
        "representative_gate": representative_gate,
        "backends": backend_summary,
        "direction_probe": summarize_direction_probe(direction_probe),
        "pick_scene_isolation": {
            "enabled": True,
            "hidden_bodies": [
                "bottle", "cup", "bowl", "demo_bottle", "demo_box"],
            "max_settle_displacement_mm":
                smoke_summary["max_target_settle_displacement_mm"],
            "unexpected_contact_trials": smoke_summary["unexpected_contact_trials"],
        },
        "out_of_domain_diagnostics": {
            "excluded_from_success_rate": True,
            "configured_y_max_m": domain_y[1], "scene_default_bottle_y_m": 0.12,
            "configuration_mismatch": 0.12 > domain_y[1], "cases": diagnostic_rows,
        },
        "yolo_executed": "yolo" in backend_summary,
        "wall_clock_s": round(time.perf_counter() - started, 2),
        "notes": [
            "Pick success requires the existing VERIFY_GRASP lift threshold of 30 mm.",
            "Out-of-domain diagnostics are excluded from all success-rate denominators.",
            "The scene default ycb_bottle y=0.12 m exceeds the configured ODD y maximum.",
            "sim_gt isolates manipulation; YOLO exercises YOLO plus RGB-D localization.",
            "The pick-only fixture hides and disables non-target cup/bottle/bowl geometry.",
        ],
    }
    _write_csv(output_dir / "smoke_trials.csv", smoke_records)
    _write_csv(output_dir / "representative_trials.csv", representative_records)
    _write_csv(output_dir / "trials.csv", records + diagnostics)
    _write_dataclass_csv(output_dir / "direction_probe.csv", direction_probe)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    render_grasp_heatmap(summary, output_dir / "bottle_grasp_heatmap.png",
                         spec.x_values_m, spec.y_values_m, ood_y_values)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary
