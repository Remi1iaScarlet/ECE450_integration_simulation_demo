"""Realtime MuJoCo viewer for the camera-driven visual grasp demo.

This is the interactive counterpart of grasp_sim_camera.py:
  1. settle the scene,
  2. move to a wrist-camera observation pose,
  3. localize the cup from wrist_cam RGB + depth (Route B),
  4. open the MuJoCo viewer,
  5. play pre-grasp -> descend -> close gripper -> lift in realtime.

On macOS, MuJoCo's passive viewer must be launched with mjpython:
    MUJOCO_GL=glfw mjpython sim/grasp_sim_viewer.py
"""
import argparse
import os
import pathlib
import sys
import time

import numpy as np

os.environ.setdefault("MUJOCO_GL", "glfw" if sys.platform == "darwin" else "egl")
import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import phase6_route_b  # noqa: E402
from grasp_sim import (  # noqa: E402
    GRIP_CLOSE,
    GRIP_OPEN,
    horizontal_grasp_frame,
    horizontal_pregrasp_point,
    ik_target,
    settle,
)

SCENE = ROOT / "sim" / "models" / "piper" / "scene_grasp.xml"
np.set_printoptions(precision=4, suppress=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run the camera-driven grasp demo in the MuJoCo viewer."
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier. 1.0 is realtime; 2.0 is twice as fast.",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=-1.0,
        help="Seconds to keep the viewer open after the sequence. Negative means until closed.",
    )
    parser.add_argument(
        "--view",
        choices=["free", "wrist", "world"],
        default="wrist",
        help="Initial viewer camera. Use 'wrist' to see the eye-in-hand camera view.",
    )
    return parser.parse_args()


def play_ctrl(model, data, viewer, ctrl, steps, label, speed):
    """Apply ctrl for a fixed number of MuJoCo steps and sync the viewer."""
    print(f"[viewer] {label}")
    step_dt = model.opt.timestep / speed
    for _ in range(steps):
        if not viewer.is_running():
            return False
        t0 = time.perf_counter()
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
        viewer.sync()
        wait = step_dt - (time.perf_counter() - t0)
        if wait > 0:
            time.sleep(wait)
    return True


def hold_viewer(viewer, seconds):
    """Keep the final pose visible until the user closes the window or time expires."""
    if seconds < 0:
        while viewer.is_running():
            viewer.sync()
            time.sleep(1.0 / 60.0)
        return
    end = time.perf_counter() + seconds
    while viewer.is_running() and time.perf_counter() < end:
        viewer.sync()
        time.sleep(1.0 / 60.0)


def set_viewer_camera(model, viewer, view_name):
    if view_name == "free":
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        return
    cam_name = "wrist_cam" if view_name == "wrist" else "world_cam"
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
    if cam_id < 0:
        raise RuntimeError(f"camera not found: {cam_name}")
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    viewer.cam.fixedcamid = cam_id


def stop_and_hold(viewer, message, hold_seconds):
    print(f"[error] {message}")
    hold_viewer(viewer, hold_seconds)


def main():
    args = parse_args()
    if args.speed <= 0:
        raise SystemExit("--speed must be positive")

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.vis.global_.offwidth = 640
    model.vis.global_.offheight = 480
    data = mujoco.MjData(model)

    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    cup = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cup")
    if home < 0 or tcp < 0 or cup < 0:
        raise RuntimeError("scene is missing required home keyframe, tcp site, or cup body")

    # Match grasp_sim_camera.py: do not reset the full keyframe here. The scene has
    # freejoint objects, and resetting the included robot keyframe can disturb their
    # initial object poses. Let MuJoCo use the XML initial qpos for objects, then
    # drive the arm toward the home ctrl.
    data.ctrl[:] = model.key_ctrl[home]
    settle(model, data, model.key_ctrl[home], 700)
    phase6_route_b.move_to_observation_pose(model, data)

    print(f"[perception] localizing cup from {phase6_route_b.DEFAULT_CAM} RGB + depth ...")
    p_cam_base, n_roi, bbox = phase6_route_b.camera_object_point(model, data)
    cup0 = data.xpos[cup].copy()
    if p_cam_base is None:
        # No ground-truth fallback: a camera-driven grasp must use a real
        # camera detection. If the cup is not detected, fail loudly so the
        # missed detection is never silently masked by ground truth.
        raise RuntimeError(
            "no cup detected by camera; cannot start camera-driven grasp. "
            "Run `MUJOCO_GL=egl python3 sim/perception.py` to inspect the wrist "
            "view and YOLO output."
        )
    print(f"[perception] bbox={[round(v, 1) for v in bbox]}  ROI points={n_roi}")
    print(f"[perception] camera-derived target={p_cam_base}")
    print(f"[perception] ground truth reference={cup0}  "
          f"|err|={np.linalg.norm(p_cam_base - cup0) * 1000:.0f} mm")

    grasp = p_cam_base.copy()
    grasp[2] = max(grasp[2], cup0[2] + 0.005)
    grasp_mat = horizontal_grasp_frame(grasp)
    pre = horizontal_pregrasp_point(grasp, grasp_mat)
    lift = grasp + np.array([0.0, 0.0, 0.15])
    ctrl = np.array(model.key_ctrl[home])

    print("[viewer] opening MuJoCo viewer. Close the window to stop.")
    try:
        viewer_ctx = mujoco.viewer.launch_passive(
            model, data, show_left_ui=True, show_right_ui=True
        )
    except RuntimeError as exc:
        if sys.platform == "darwin" and "mjpython" in str(exc):
            raise SystemExit(
                "On macOS, run this script with:\n"
                "  MUJOCO_GL=glfw mjpython sim/grasp_sim_viewer.py"
            ) from exc
        raise

    with viewer_ctx as viewer:
        set_viewer_camera(model, viewer, args.view)
        viewer.sync()
        hold_viewer(viewer, 1.0)

        q, ok = ik_target(model, data, tcp, pre)
        print(f"[ik] pre-grasp position ok={ok} q={q}")
        if not ok:
            stop_and_hold(viewer, "pre-grasp position IK did not converge", args.hold)
            return
        ctrl[:6] = q
        ctrl[6] = GRIP_OPEN
        if not play_ctrl(model, data, viewer, ctrl, 700, "move to pre-grasp", args.speed):
            return

        q, ok = ik_target(model, data, tcp, grasp, grasp_mat)
        print(f"[ik] grasp ok={ok} q={q}")
        if not ok:
            stop_and_hold(viewer, "horizontal grasp IK did not converge", args.hold)
            return
        ctrl[:6] = q
        ctrl[6] = GRIP_OPEN
        if not play_ctrl(model, data, viewer, ctrl, 700, "descend to grasp", args.speed):
            return
        print(f"[state] at-grasp tcp={data.site_xpos[tcp]} cup={data.xpos[cup]}")

        ctrl[6] = GRIP_CLOSE
        if not play_ctrl(model, data, viewer, ctrl, 600, "close gripper", args.speed):
            return

        q, ok = ik_target(model, data, tcp, lift, grasp_mat)
        print(f"[ik] lift ok={ok} q={q}")
        if not ok:
            stop_and_hold(viewer, "horizontal lift IK did not converge", args.hold)
            return
        ctrl[:6] = q
        if not play_ctrl(model, data, viewer, ctrl, 900, "lift object", args.speed):
            return

        cup_now = data.xpos[cup].copy()
        rose = cup_now[2] - cup0[2]
        print(f"[result] lift cup={cup_now} rose={rose * 1000:.0f} mm -> "
              f"{'LIFTED' if rose > 0.03 else 'NOT lifted'}")
        hold_viewer(viewer, args.hold)


if __name__ == "__main__":
    main()
