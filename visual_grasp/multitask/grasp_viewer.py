"""Interactive MuJoCo viewer for the current bottle grasp pipeline.

On macOS this module must be launched with ``mjpython`` so the passive viewer
runs on the required main thread::

    MUJOCO_GL=glfw mjpython -m multitask.grasp_viewer
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import Sequence

os.environ.setdefault("MUJOCO_GL", "glfw" if sys.platform == "darwin" else "egl")

import mujoco
import mujoco.viewer

from .executor import TaskExecutor
from .grasp_heatmap import (
    DEFAULT_X_VALUES,
    DOMAIN_IN,
    GraspTrialCase,
    _body_geom_ids,
    _configured_world,
    _domain_ranges,
    _move_body,
    reset_target_for_trial,
)
from .perception_benchmark import E2E_INTERIOR_Y
from sim.constrained_ik import clear_constrained_ik_cache


REPRESENTATIVE_POSITIONS = (
    (0.34, -0.160),
    (0.34, -0.025),
    (0.38, -0.070),
    (0.42, -0.160),
    (0.42, -0.025),
)


class ViewerClosed(RuntimeError):
    """Raised internally to stop the pipeline when the user closes the viewer."""


@dataclass(frozen=True)
class ViewerCase:
    case_id: str
    x_m: float
    y_m: float
    yaw_deg: float


@dataclass(frozen=True)
class ViewerCaseResult:
    case_id: str
    success: bool
    failed_state: str | None
    lift_height_mm: float | None


@dataclass(frozen=True)
class ViewerOptions:
    preset: str = "representative"
    custom_positions: tuple[tuple[float, float], ...] = ()
    yaw_deg: float = 0.0
    backend: str = "yolo"
    speed: float = 1.0
    pause_s: float = 1.0
    hold_s: float = -1.0
    view: str = "world"
    scene_layout: str = "demo"
    motion_mode: str = "continuous"


def build_viewer_cases(
        preset: str,
        custom_positions: Sequence[tuple[float, float]] = (),
        yaw_deg: float = 0.0) -> tuple[ViewerCase, ...]:
    """Build deterministic demonstration cases; custom positions override preset."""
    if custom_positions:
        positions = tuple((float(x), float(y)) for x, y in custom_positions)
        prefix = "custom"
    elif preset == "representative":
        positions = REPRESENTATIVE_POSITIONS
        prefix = "representative"
    elif preset == "full-grid":
        positions = tuple((float(x), float(y))
                          for x in DEFAULT_X_VALUES for y in E2E_INTERIOR_Y)
        prefix = "grid"
    else:
        raise ValueError(f"unknown viewer preset: {preset}")
    return tuple(ViewerCase(
        case_id=f"{prefix}-{index:02d}", x_m=x, y_m=y,
        yaw_deg=float(yaw_deg))
        for index, (x, y) in enumerate(positions, start=1))


def validate_cases(cases: Sequence[ViewerCase], domain_x, domain_y):
    if not cases:
        raise ValueError("at least one viewer case is required")
    for case in cases:
        if not (domain_x[0] <= case.x_m <= domain_x[1] and
                domain_y[0] <= case.y_m <= domain_y[1]):
            raise ValueError(
                f"case {case.case_id} xy=({case.x_m}, {case.y_m}) is outside "
                f"the operating domain x={domain_x}, y={domain_y}")


def _set_view(viewer, model, view):
    if view == "free":
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        return
    camera_name = "world_cam" if view == "world" else "wrist_cam"
    camera_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if camera_id < 0:
        raise RuntimeError(f"camera not found: {camera_name}")
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    viewer.cam.fixedcamid = camera_id


class _ViewerPlayback:
    def __init__(self, viewer, model, speed):
        self.viewer = viewer
        self.step_dt = float(model.opt.timestep) / speed
        self.last_sync = time.perf_counter()
        self.next_step_at = self.last_sync

    def after_step(self):
        if not self.viewer.is_running():
            raise ViewerClosed
        self.next_step_at += self.step_dt
        remaining = self.next_step_at - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        elif remaining < -0.25:
            # Do not accumulate an unbounded catch-up burst after slow inference.
            self.next_step_at = time.perf_counter()
        now = time.perf_counter()
        if now - self.last_sync >= 1.0 / 60.0:
            self.viewer.sync()
            self.last_sync = now

    def pause(self, seconds):
        deadline = None if seconds < 0 else time.perf_counter() + seconds
        while self.viewer.is_running() and (
                deadline is None or time.perf_counter() < deadline):
            self.viewer.sync()
            time.sleep(1.0 / 60.0)
        if not self.viewer.is_running():
            raise ViewerClosed


def _failed_state(summary):
    if summary["success"]:
        return None
    return next((row["state"] for row in reversed(summary["states"])
                 if not row["success"]), None)


def _to_trial(case: ViewerCase) -> GraspTrialCase:
    return GraspTrialCase(
        case_id=case.case_id,
        grid_x_m=case.x_m,
        grid_y_m=case.y_m,
        actual_x_m=case.x_m,
        actual_y_m=case.y_m,
        yaw_deg=case.yaw_deg,
        repeat_index=1,
        domain_class=DOMAIN_IN,
    )


def reset_demo_scene(world, cfg, case: ViewerCase):
    """Reset the yellow target while retaining the blue bottle and open box."""
    mujoco.mj_resetData(world.model, world.data)
    home = mujoco.mj_name2id(
        world.model, mujoco.mjtObj.mjOBJ_KEY, "home")
    world.ctrl[:] = world.model.key_ctrl[home]
    world.ready_ctrl = world.model.key_ctrl[home].copy()
    world.data.ctrl[:] = world.ctrl
    world.frames.clear()
    world.clearance_minima.clear()
    if mujoco.mj_name2id(
            world.model, mujoco.mjtObj.mjOBJ_BODY, "cup") >= 0:
        _move_body(world, "cup", (2.0, -2.0, 0.20))
    for geom_id in _body_geom_ids(world.model, "bowl"):
        world.model.geom_contype[geom_id] = 0
        world.model.geom_conaffinity[geom_id] = 0
        world.model.geom_rgba[geom_id, 3] = 0.0
    target_body = cfg["objects"]["body_of"]["bottle"]
    _move_body(world, target_body, (case.x_m, case.y_m, 0.20), case.yaw_deg)
    mujoco.mj_forward(world.model, world.data)
    clear_constrained_ik_cache()
    world.settle(300)


def run_viewer(options: ViewerOptions) -> list[ViewerCaseResult]:
    if options.speed <= 0:
        raise ValueError("speed must be positive")
    if options.pause_s < 0:
        raise ValueError("pause must be non-negative")
    if options.backend not in ("yolo", "sim_gt"):
        raise ValueError(f"unknown perception backend: {options.backend}")
    cases = build_viewer_cases(
        options.preset, options.custom_positions, options.yaw_deg)
    cfg, world, canonical = _configured_world(
        options.backend, "piper_real", "radial_single")
    cfg.setdefault("motion", {})["execution_mode"] = options.motion_mode
    domain_x, domain_y = _domain_ranges(cfg)
    validate_cases(cases, domain_x, domain_y)
    results = []

    print(f"[viewer] model={canonical} backend={options.backend} "
          f"bottle_conf={cfg['detector']['confidence_threshold']} cases={len(cases)}")
    print("[viewer] opening MuJoCo viewer; close the window to stop.")
    try:
        context = mujoco.viewer.launch_passive(
            world.model, world.data, show_left_ui=True, show_right_ui=True)
    except RuntimeError as exc:
        if sys.platform == "darwin" and "mjpython" in str(exc):
            raise SystemExit(
                "On macOS run: MUJOCO_GL=glfw mjpython -m multitask.grasp_viewer"
            ) from exc
        raise

    try:
        with context as viewer:
            _set_view(viewer, world.model, options.view)
            playback = _ViewerPlayback(viewer, world.model, options.speed)
            world.after_step = playback.after_step
            viewer.sync()
            playback.pause(min(1.0, options.pause_s))
            for index, case in enumerate(cases, start=1):
                print(f"\n[case {index:02d}/{len(cases):02d}] {case.case_id} "
                      f"xy=({case.x_m:.3f}, {case.y_m:.3f}) yaw={case.yaw_deg:.1f}")
                if options.scene_layout == "demo":
                    reset_demo_scene(world, cfg, case)
                else:
                    reset_target_for_trial(world, cfg, "bottle", _to_trial(case))
                viewer.sync()
                summary = TaskExecutor(world, cfg, verbose=True).pick("bottle")
                result = ViewerCaseResult(
                    case_id=case.case_id,
                    success=bool(summary["success"]),
                    failed_state=_failed_state(summary),
                    lift_height_mm=summary["data"].get("rose_mm"),
                )
                results.append(result)
                print(f"[result] {'PASS' if result.success else 'FAIL'} "
                      f"failed_state={result.failed_state or '-'} "
                      f"lift={result.lift_height_mm} mm")
                if index < len(cases):
                    playback.pause(options.pause_s)
            print(f"\n[viewer] completed {sum(r.success for r in results)}/"
                  f"{len(results)} cases")
            playback.pause(options.hold_s)
    except ViewerClosed:
        print(f"\n[viewer] window closed; stopped after {len(results)}/{len(cases)} cases")
    finally:
        world.after_step = None
    return results


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Visualize the current bottle grasp pipeline in MuJoCo.")
    parser.add_argument("--preset", choices=("representative", "full-grid"),
                        default="representative")
    parser.add_argument("--position", type=float, nargs=2, action="append",
                        metavar=("X", "Y"), default=[])
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--backend", choices=("yolo", "sim_gt"), default="yolo")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--pause", type=float, default=1.0)
    parser.add_argument("--hold", type=float, default=-1.0)
    parser.add_argument("--view", choices=("free", "world", "wrist"), default="world")
    parser.add_argument("--scene-layout", choices=("demo", "isolated"),
                        default="demo")
    parser.add_argument("--motion-mode", choices=("continuous", "legacy"),
                        default="continuous")
    args = parser.parse_args(argv)
    return ViewerOptions(
        preset=args.preset,
        custom_positions=tuple(tuple(position) for position in args.position),
        yaw_deg=args.yaw_deg,
        backend=args.backend,
        speed=args.speed,
        pause_s=args.pause,
        hold_s=args.hold,
        view=args.view,
        scene_layout=args.scene_layout,
        motion_mode=args.motion_mode,
    )


def main(argv=None):
    try:
        run_viewer(parse_args(argv))
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()
