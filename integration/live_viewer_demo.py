#!/usr/bin/env python3
"""Live MuJoCo viewer version of run_demo.py -- for a machine with a real
display. run_demo.py renders offscreen (EGL) and saves a GIF/MP4; this
script instead opens an interactive window you can orbit/zoom while the
LLM-selected task runs.

Requirements this repo cannot verify for you (no display on the machine
this repo was built on):
  - MUJOCO_GL=glfw (not egl) and a working X/Wayland display.
  - On macOS, MuJoCo's passive viewer must run on the main thread, so you'd
    launch this with `mjpython` instead of `python3` (same constraint
    documented in visual_grasp/DEMOS.md and multitask/grasp_viewer.py). On
    Linux -- almost certainly what a "cloud host with a screen" is -- plain
    `python3` is correct; this script never invokes mjpython itself.

How it works: multitask/world.py's SimWorld has a documented
`after_step` hook ("optional passive-viewer sync hook") that
multitask/grasp_viewer.py already uses for exactly this purpose. This
script passes a custom `sim_world_factory` into bridge.run_bridge() (the
same injection point tests/test_bridge.py exercises) that opens the viewer
right after the world is built and wires `world.after_step` to a
60fps-throttled, real-time-paced sync -- mirroring
multitask/grasp_viewer.py's `_ViewerPlayback` class instead of reinventing
the pacing logic.

Caveat: SimWorld.load() calls world.settle(700) internally, before
returning -- so the initial ~2s of objects dropping onto the table happens
before the viewer exists and won't be visible live. The viewer opens once
the world is ready and shows the actual pick/place motion, which is the
part worth watching anyway.

Usage: same three command sources as run_demo.py.

    MUJOCO_GL=glfw python3 integration/live_viewer_demo.py --command "put the cup in the bowl"
    MUJOCO_GL=glfw python3 integration/live_viewer_demo.py --llm-json '{"intent": "pick", "parameters": {"source_label": "bottle"}}'
    MUJOCO_GL=glfw python3 integration/live_viewer_demo.py --from-queue
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "llm"))
sys.path.insert(0, str(REPO_ROOT / "visual_grasp"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from oh_bridge_mock import UnroutableCommand, llm_item_to_bridge_command  # noqa: E402
from run_demo import (  # noqa: E402
    _from_command_text,
    _from_queue,
    _mark_queue_item,
    _validated_llm_output,
)


class ViewerClosed(RuntimeError):
    """Raised from the after_step hook when the user closes the window."""


class _ViewerPlayback:
    """Real-time-paced, 60fps-throttled sync. Mirrors
    multitask/grasp_viewer.py's _ViewerPlayback -- same pacing logic, not
    reinvented, since that one is already tuned for this codebase's models.
    """

    def __init__(self, viewer, model, speed: float = 1.0):
        self.viewer = viewer
        self.step_dt = float(model.opt.timestep) / speed
        self.last_sync = time.perf_counter()
        self.next_step_at = self.last_sync

    def __call__(self) -> None:
        if not self.viewer.is_running():
            raise ViewerClosed
        self.next_step_at += self.step_dt
        remaining = self.next_step_at - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        elif remaining < -0.25:
            self.next_step_at = time.perf_counter()
        now = time.perf_counter()
        if now - self.last_sync >= 1.0 / 60.0:
            self.viewer.sync()
            self.last_sync = now


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--from-queue", action="store_true")
    source.add_argument("--command", help="natural-language command, sent to the real LLM mapper")
    source.add_argument("--llm-json", help='{"intent": ..., "parameters": {...}}, no LLM call')
    parser.add_argument("--api-base", default="http://127.0.0.1:8000")
    parser.add_argument("--perception", default="sim_gt", choices=["sim_gt", "yolo"])
    parser.add_argument("--model", default="real", choices=["menagerie", "real"])
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    parser.add_argument("--hold", type=float, default=8.0,
                         help="seconds to keep the window open after the task finishes")
    args = parser.parse_args()

    try:
        import mujoco.viewer
    except ImportError as exc:
        parser.error(f"mujoco.viewer unavailable: {exc}")

    queue_item_id = None
    if args.from_queue:
        llm_output, queue_item_id = _from_queue(args.api_base)
    elif args.command:
        llm_output = _from_command_text(args.command)
    else:
        try:
            parsed = json.loads(args.llm_json)
        except json.JSONDecodeError as exc:
            parser.error(f"--llm-json is not valid JSON: {exc}")
        llm_output = _validated_llm_output(parsed.get("intent"), parsed.get("parameters", {}))

    try:
        bridge_command = llm_item_to_bridge_command(
            llm_output,
            request_id=f"live_{queue_item_id or 'local'}",
            backend="sim_mujoco",
            perception=args.perception,
            model=args.model,
        )
    except UnroutableCommand as exc:
        print(str(exc), file=sys.stderr)
        if queue_item_id is not None:
            _mark_queue_item(args.api_base, queue_item_id, "failed")
        return 2

    viewer_ref: dict = {}

    def viewer_world_factory(**kwargs):
        from multitask.world import SimWorld

        world = SimWorld.load(**kwargs)
        try:
            context = mujoco.viewer.launch_passive(world.model, world.data)
        except RuntimeError as exc:
            if sys.platform == "darwin" and "mjpython" in str(exc):
                raise SystemExit(
                    "On macOS this needs: MUJOCO_GL=glfw mjpython integration/live_viewer_demo.py ..."
                ) from exc
            raise
        viewer = context.__enter__()
        viewer_ref["context"] = context
        viewer_ref["viewer"] = viewer
        world.after_step = _ViewerPlayback(viewer, world.model, args.speed)
        return world

    from multitask import bridge  # visual_grasp/multitask/bridge.py

    try:
        result = bridge.run_bridge(
            bridge_command,
            no_evidence=True,
            sim_world_factory=viewer_world_factory,
        )
    except ViewerClosed:
        print("viewer window closed; stopped early", file=sys.stderr)
        if queue_item_id is not None:
            _mark_queue_item(args.api_base, queue_item_id, "failed")
        return 130
    finally:
        viewer = viewer_ref.get("viewer")
        context = viewer_ref.get("context")
        if viewer is not None and viewer.is_running() and args.hold > 0:
            print(f"task finished; holding window open for {args.hold:.0f}s "
                  f"(close it or Ctrl+C to exit early)")
            deadline = time.perf_counter() + args.hold
            while viewer.is_running() and time.perf_counter() < deadline:
                viewer.sync()
                time.sleep(1.0 / 60.0)
        if context is not None:
            context.__exit__(None, None, None)

    print(json.dumps(result, ensure_ascii=False, indent=2, default=str))

    if queue_item_id is not None:
        _mark_queue_item(args.api_base, queue_item_id, "executed" if result.get("success") else "failed")

    return 0 if result.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
