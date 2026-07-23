"""Repeatable MuJoCo benchmark for the DR1 robot-task quality specification.

The DR1 validation method calls for a 100-trial Pick-and-Place benchmark. This
runner keeps the MuJoCo model loaded, resets the simulation between trials,
randomizes the cup inside a documented safe tabletop region, and writes both
trial-level CSV data and a compact JSON summary.

Examples (run from visual_grasp/):
    MUJOCO_GL=cgl python -m multitask.benchmark --backend sim_gt --trials 100
    MUJOCO_GL=cgl python -m multitask.benchmark --backend yolo --trials 100
"""
import argparse
import csv
import json
import os
import pathlib
import statistics
import time
import warnings
from collections import Counter

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np

from .config import load_config
from .executor import TaskExecutor
from . import grasp_planner
from sim.constrained_ik import clear_constrained_ik_cache
from .world import SimWorld


DEFAULT_OUTPUT = pathlib.Path(__file__).resolve().parent / "benchmark_results"
ROOT = pathlib.Path(__file__).resolve().parent.parent


def _freejoint_qpos_address(model, body_name):
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body < 0 or model.body_jntnum[body] < 1:
        raise ValueError(f"body {body_name!r} has no free joint")
    joint = model.body_jntadr[body]
    return int(model.jnt_qposadr[joint])


def _reset_trial(world, cup_xy, cup_yaw_deg=0.0):
    """Reset arm/objects, move the cup, and settle before one measured trial."""
    mujoco.mj_resetData(world.model, world.data)
    home = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_KEY, "home")
    world.ctrl[:] = world.model.key_ctrl[home]
    world.data.ctrl[:] = world.ctrl
    world.frames.clear()

    # Match SimWorld.load(): hide the synthetic placeholder bottle when the YCB
    # bottle is present, so it cannot become an unintended detection/obstacle.
    green = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_BODY, "bottle")
    if green >= 0:
        green_addr = _freejoint_qpos_address(world.model, "bottle")
        world.data.qpos[green_addr:green_addr + 3] = [2.0, 2.0, 0.20]

    cup_addr = _freejoint_qpos_address(world.model, "cup")
    world.data.qpos[cup_addr:cup_addr + 3] = [cup_xy[0], cup_xy[1], 0.20]
    half = np.deg2rad(cup_yaw_deg) / 2.0
    world.data.qpos[cup_addr + 3:cup_addr + 7] = [np.cos(half), 0.0, 0.0, np.sin(half)]
    mujoco.mj_forward(world.model, world.data)
    world.settle(700)


def _state_duration(summary, state):
    return round(sum(s.get("duration_ms", 0.0) for s in summary["states"]
                     if s["state"] == state), 1)


def _failed_state(summary):
    if summary["success"]:
        return ""
    return next((s["state"] for s in reversed(summary["states"]) if not s["success"]), "")


def _failure_reason(summary):
    if summary["success"]:
        return ""
    return next((s.get("reason", "STATE_FAILED") for s in reversed(summary["states"])
                 if not s["success"]), "")


def generate_cases(trials, seed, x_range, y_range, yaw_range=(0.0, 360.0)):
    """Generate immutable cases once so every strategy/backend sees identical inputs."""
    rng = np.random.default_rng(seed)
    return [{"case_id": i + 1, "x": float(x), "y": float(y), "yaw_deg": float(yaw)}
            for i, (x, y, yaw) in enumerate(zip(
                rng.uniform(*x_range, size=trials),
                rng.uniform(*y_range, size=trials),
                rng.uniform(*yaw_range, size=trials)))]


def generate_xy_grid(x_range=(0.34, 0.42), y_range=(-0.16, -0.06), side=5, yaw_deg=0.0):
    """Deterministic fixed-yaw grid for IK coverage; never mixes yaw with XY."""
    return [{"case_id": i + 1, "x": float(x), "y": float(y), "yaw_deg": float(yaw_deg)}
            for i, (x, y) in enumerate((x, y) for x in np.linspace(*x_range, side)
                                       for y in np.linspace(*y_range, side))]


def generate_yaw_detection_cases(points, yaws=range(0, 360, 45)):
    """Repeat identical XY points at each yaw for perception-only experiments."""
    return [{"case_id": i + 1, "x": float(x), "y": float(y), "yaw_deg": float(yaw)}
            for i, (x, y, yaw) in enumerate((x, y, yaw) for x, y in points for yaw in yaws)]


def diagnose_edge_cases(output_path=None):
    """Deterministic piper_real IK/collision reproduction for grid cases 23-25."""
    cfg = load_config()
    msel = cfg["model"]["real"]
    for section in ("grasp", "place", "scene", "detector"):
        if isinstance(msel.get(section), dict):
            cfg[section].update(msel[section])
    world = SimWorld.load(scene=ROOT / msel["scene"], record=False,
                          use_analytic_ik=True)
    target_body_id = mujoco.mj_name2id(
        world.model, mujoco.mjtObj.mjOBJ_BODY, "cup")
    clear_constrained_ik_cache()
    rows = []
    for case_id, y in ((23, -0.11), (24, -0.085), (25, -0.06)):
        started = time.perf_counter()
        ranked = grasp_planner.plan(
            [0.42, y, cfg["scene"]["table_top_z"]], world, cfg,
            target_body_id=target_body_id)
        rows.append({
            "case_id": case_id, "x": 0.42, "y": y,
            "elapsed_s": round(time.perf_counter() - started, 3),
            "feasible_candidates": [c.candidate_id for c in ranked if c.feasible],
            "candidates": [c.public() for c in ranked],
        })
    report = {"model": "piper_real", "suite": "edge_collision_diagnostic",
              "cases": rows}
    if output_path is not None:
        pathlib.Path(output_path).write_text(
            json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    return report


def _mean(values):
    values = [float(v) for v in values if v is not None]
    return round(statistics.fmean(values), 2) if values else None


def _percentile(values, percentile):
    values = sorted(float(v) for v in values if v is not None)
    if not values:
        return None
    return round(float(np.percentile(values, percentile)), 2)


def canonical_model_name(model_name):
    if model_name == "real":
        warnings.warn("model name 'real' is deprecated; use 'piper_real'",
                      DeprecationWarning, stacklevel=2)
        return "piper_real"
    return model_name


def run(backend, model_name, trials, seed, output_dir,
        x_range=(0.34, 0.42), y_range=(-0.16, -0.06),
        yaw_range=(0.0, 360.0), strategy="radial_single", cases=None):
    cfg = load_config()
    cfg["detector"]["backend"] = backend
    cfg["grasp"]["strategy"] = strategy
    canonical_model = canonical_model_name(model_name)
    config_key = "real" if canonical_model == "piper_real" else canonical_model
    msel = cfg.get("model", {}).get(config_key, {})
    scene_path = (ROOT / msel["scene"]) if msel.get("scene") else None
    analytic_ik = bool(msel.get("analytic_ik", False))
    if msel.get("scan_poses"):
        cfg["detector"]["scan_poses"] = msel["scan_poses"]
    for section in ("grasp", "place", "scene", "detector"):
        if isinstance(msel.get(section), dict):
            cfg[section].update(msel[section])
    world = SimWorld.load(scene=scene_path, record=False,
                          use_analytic_ik=analytic_ik)
    cases = cases or generate_cases(trials, seed, x_range, y_range, yaw_range)
    if len(cases) != trials:
        raise ValueError("case count must equal trials")

    rows = []
    benchmark_started = time.perf_counter()
    for i, case in enumerate(cases, start=1):
        x, y, yaw = case["x"], case["y"], case["yaw_deg"]
        _reset_trial(world, (x, y), yaw)
        executor = TaskExecutor(world, cfg, verbose=False)
        summary = executor.place_into("cup", container_label="bowl")
        data = summary["data"]
        rows.append({
            "trial": i,
            "model": canonical_model,
            "backend": backend,
            "strategy": strategy,
            "seed": seed,
            "cup_x_m": round(float(x), 5),
            "cup_y_m": round(float(y), 5),
            "cup_yaw_deg": round(float(yaw), 3),
            "success": int(bool(summary["success"])),
            "failed_state": _failed_state(summary),
            "failure_reason": _failure_reason(summary),
            "candidate_count": data.get("candidate_count"),
            "selected_candidate_id": data.get("selected_candidate_id"),
            "selected_direction_deg": data.get("selected_direction_deg"),
            "selected_joint_margin_rad": data.get("selected_joint_margin_rad"),
            "failed_candidate_ids_json": json.dumps(data.get("failed_candidate_ids", [])),
            "task_duration_ms": data.get("task_duration_ms"),
            "detect_source_ms": _state_duration(summary, "DETECT_SOURCE"),
            "execute_pick_ms": _state_duration(summary, "EXECUTE_PICK"),
            "detect_target_ms": _state_duration(summary, "DETECT_TARGET"),
            "execute_place_ms": _state_duration(summary, "EXECUTE_PLACE"),
            "source_center_error_mm": data.get("source_center_error_mm"),
            "target_center_error_mm": data.get("target_center_error_mm"),
            "lift_height_mm": data.get("rose_mm"),
            "placement_error_mm": data.get("place_dxy_mm"),
            "state_log_json": json.dumps(summary["states"], ensure_ascii=False),
        })
        print(f"[{canonical_model}/{backend}/{strategy}] {i:03d}/{trials}: "
              f"{'PASS' if summary['success'] else 'FAIL'} "
              f"duration={data.get('task_duration_ms')} ms "
              f"place_error={data.get('place_dxy_mm')} mm")

    output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{canonical_model}_{backend}_{strategy}_{trials}"
    csv_path = output_dir / f"{stem}_trials.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    successes = sum(r["success"] for r in rows)
    successful_rows = [r for r in rows if r["success"]]
    summary = {
        "backend": backend,
        "strategy": strategy,
        "model": canonical_model,
        "trials": trials,
        "seed": seed,
        "successes": successes,
        "success_rate_pct": round(100.0 * successes / trials, 2),
        "dr1_pick_place_target_pct": 85.0,
        "dr1_success_target_met": successes / trials >= 0.85,
        "mean_task_duration_ms": _mean(r["task_duration_ms"] for r in rows),
        "p95_task_duration_ms": _percentile((r["task_duration_ms"] for r in rows), 95),
        "mean_detect_source_ms": _mean(r["detect_source_ms"] for r in rows),
        "p95_detect_source_ms": _percentile((r["detect_source_ms"] for r in rows), 95),
        "mean_source_center_error_mm": _mean(r["source_center_error_mm"] for r in rows),
        "p95_source_center_error_mm": _percentile(
            (r["source_center_error_mm"] for r in rows), 95),
        "mean_placement_error_mm": _mean(r["placement_error_mm"] for r in rows),
        "p95_placement_error_mm": _percentile((r["placement_error_mm"] for r in rows), 95),
        "mean_successful_placement_error_mm": _mean(
            r["placement_error_mm"] for r in successful_rows),
        "p95_successful_placement_error_mm": _percentile(
            (r["placement_error_mm"] for r in successful_rows), 95),
        "mean_lift_height_mm": _mean(r["lift_height_mm"] for r in rows),
        "failure_state_counts": dict(Counter(
            r["failed_state"] for r in rows if not r["success"])),
        "failure_reason_counts": dict(Counter(
            r["failure_reason"] for r in rows if not r["success"])),
        "success_rate_by_yaw_quadrant_pct": {
            f"{lo}-{lo + 90}": round(100.0 * sum(
                r["success"] for r in rows if lo <= r["cup_yaw_deg"] < lo + 90) /
                max(1, sum(1 for r in rows if lo <= r["cup_yaw_deg"] < lo + 90)), 2)
            for lo in (0, 90, 180, 270)},
        "success_rate_by_x_half_pct": {
            label: round(100.0 * sum(r["success"] for r in rows if pred(r)) /
                         max(1, sum(1 for r in rows if pred(r))), 2)
            for label, pred in {
                "near": lambda r: r["cup_x_m"] < (x_range[0] + x_range[1]) / 2,
                "far": lambda r: r["cup_x_m"] >= (x_range[0] + x_range[1]) / 2,
            }.items()},
        "success_rate_by_y_half_pct": {
            label: round(100.0 * sum(r["success"] for r in rows if pred(r)) /
                         max(1, sum(1 for r in rows if pred(r))), 2)
            for label, pred in {
                "negative": lambda r: r["cup_y_m"] < (y_range[0] + y_range[1]) / 2,
                "positive": lambda r: r["cup_y_m"] >= (y_range[0] + y_range[1]) / 2,
            }.items()},
        "wall_clock_s": round(time.perf_counter() - benchmark_started, 2),
        "test_region": {"cup_x_m": list(x_range), "cup_y_m": list(y_range),
                        "cup_yaw_deg": list(yaw_range)},
        "notes": [
            "Task latency is host-side simulated task execution time, not DR1 ROS2/HDC bridge latency.",
            "sim_gt isolates manipulation control; yolo exercises YOLO plus RGB-D localization.",
            "Only MuJoCo is tested, so the DR1 Gazebo/MuJoCo engine-delta criterion is not assessed.",
        ],
    }
    json_path = output_dir / f"{stem}_summary.json"
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return csv_path, json_path


def run_matrix(backends, strategies, model_name, trials, seed, output_dir,
               x_range, y_range, yaw_range):
    cases = generate_cases(trials, seed, x_range, y_range, yaw_range)
    outputs = []
    for backend in backends:
        for strategy in strategies:
            outputs.append(run(backend, model_name, trials, seed, output_dir,
                               x_range, y_range, yaw_range, strategy, cases))
    return outputs


def main():
    parser = argparse.ArgumentParser(description="Run the DR1 100-trial MuJoCo benchmark")
    parser.add_argument("--suite", choices=["execution", "edge_collision_diagnostic",
                                             "grasp_heatmap", "perception_calibration",
                                             "perception_reevaluate", "perception_e2e"],
                        default="execution")
    parser.add_argument("--backend", choices=["sim_gt", "yolo"])
    parser.add_argument("--backends", nargs="+", choices=["sim_gt", "yolo"])
    parser.add_argument("--strategy", choices=["radial_single", "multi_candidate_v1",
                                                "multi_candidate_v1_1"],
                        default="radial_single")
    parser.add_argument("--strategies", nargs="+",
                        choices=["radial_single", "multi_candidate_v1", "multi_candidate_v1_1"])
    parser.add_argument("--model", choices=["menagerie", "piper_real", "real"],
                        default="piper_real")
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--seed", type=int, default=20260712)
    parser.add_argument("--x-range", type=float, nargs=2, metavar=("MIN", "MAX"),
                        default=(0.34, 0.42))
    parser.add_argument("--y-range", type=float, nargs=2, metavar=("MIN", "MAX"),
                        default=(-0.16, -0.06))
    parser.add_argument("--yaw-range", type=float, nargs=2, metavar=("MIN", "MAX"),
                        default=(0.0, 360.0))
    parser.add_argument("--output-dir", type=pathlib.Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--target", default="bottle")
    parser.add_argument("--grid-repeats", type=int, default=10)
    parser.add_argument("--xy-jitter-mm", type=float, default=3.0)
    parser.add_argument("--calibration-summary", type=pathlib.Path)
    parser.add_argument("--observations-csv", type=pathlib.Path)
    args = parser.parse_args()
    if args.suite == "edge_collision_diagnostic":
        report = diagnose_edge_cases()
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return
    if args.suite == "grasp_heatmap":
        from .grasp_heatmap import GraspGridSpec, run_grasp_heatmap_suite
        backends = tuple(args.backends or ([args.backend] if args.backend else ["sim_gt", "yolo"]))
        spec = GraspGridSpec(
            target_label=args.target, repeats_per_cell=args.grid_repeats,
            xy_jitter_m=args.xy_jitter_mm / 1000.0, seed=args.seed,
            model=args.model, backends=backends)
        run_grasp_heatmap_suite(spec, args.output_dir, args.strategy)
        return
    if args.suite == "perception_calibration":
        from .perception_benchmark import (
            PerceptionBenchmarkSpec, run_perception_calibration)
        run_perception_calibration(PerceptionBenchmarkSpec(
            target_label=args.target, positive_repeats=args.grid_repeats,
            seed=args.seed, model=args.model), args.output_dir)
        return
    if args.suite == "perception_reevaluate":
        if args.observations_csv is None:
            parser.error("--observations-csv is required for perception_reevaluate")
        from .perception_benchmark import reevaluate_calibration
        reevaluate_calibration(args.observations_csv, args.output_dir)
        return
    if args.suite == "perception_e2e":
        if args.calibration_summary is None:
            parser.error("--calibration-summary is required for perception_e2e")
        from .perception_benchmark import run_perception_e2e
        run_perception_e2e(args.output_dir, args.calibration_summary,
                           model=args.model, seed=args.seed)
        return
    backends = args.backends or ([args.backend] if args.backend else None)
    if not backends:
        parser.error("one of --backend or --backends is required")
    strategies = args.strategies or [args.strategy]
    run_matrix(backends, strategies, args.model, args.trials, args.seed,
               args.output_dir, tuple(args.x_range), tuple(args.y_range),
               tuple(args.yaw_range))


if __name__ == "__main__":
    main()
