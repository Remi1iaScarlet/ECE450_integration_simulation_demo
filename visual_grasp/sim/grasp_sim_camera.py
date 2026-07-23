"""Phase 6 — camera-driven visual grasp (Route B end-to-end).

Unlike grasp_sim.py (which used the ground-truth cup pose), this gets the 3D target
ENTIRELY from the wrist camera: wrist_cam RGB -> YOLO bbox -> wrist_cam depth -> ROI
back-projection -> camera frame -> base frame via base_T_camera (sim hand-eye
calibration). Then runs the same pre-grasp/descend/close/lift sequence and checks
the cup is lifted.

Headless: requires MUJOCO_GL=egl.
    MUJOCO_GL=egl python3 sim/grasp_sim_camera.py
"""
import os
import pathlib
import sys

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import phase6_route_b  # noqa: E402  (camera_object_point)
from grasp_sim import (  # noqa: E402
    GRIP_OPEN,
    GRIP_CLOSE,
    horizontal_grasp_frame,
    horizontal_pregrasp_point,
    ik_target,
    settle,
)

OUT = ROOT / "sim" / "evidence"
SCENE = ROOT / "sim" / "models" / "piper" / "scene_grasp.xml"
np.set_printoptions(precision=4, suppress=True)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.vis.global_.offwidth = 640
    model.vis.global_.offheight = 480
    data = mujoco.MjData(model)
    tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    cup = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cup")
    renderer = mujoco.Renderer(model, height=480, width=640)
    frames = []

    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    data.ctrl[:] = model.key_ctrl[home]
    settle(model, data, model.key_ctrl[home], 700)
    phase6_route_b.move_to_observation_pose(model, data)

    # --- target ENTIRELY from the wrist camera (no ground truth) ---
    p_cam_base, n, bbox = phase6_route_b.camera_object_point(model, data)
    gt = data.xpos[cup].copy()
    if p_cam_base is None:
        # No ground-truth fallback: the target must come from a real camera
        # detection, so fail loudly rather than grasp a hidden/guessed point.
        raise RuntimeError("no cup detected by camera; cannot run camera-driven grasp")
    print(f"camera-derived target (BASE): {p_cam_base}  [{n} ROI pts]")
    print(f"ground-truth (for ref only):  {gt}  "
          f"|err|={np.linalg.norm(p_cam_base-gt)*1000:.0f} mm")

    cup0 = gt
    grasp = p_cam_base.copy()
    grasp[2] = max(grasp[2], cup0[2] + 0.005)   # don't aim below cup center
    grasp_mat = horizontal_grasp_frame(grasp)
    pre = horizontal_pregrasp_point(grasp, grasp_mat)
    lift = grasp + np.array([0, 0, 0.15])

    ctrl = np.array(model.key_ctrl[home])

    q, ok = ik_target(model, data, tcp, pre)
    if not ok:
        raise RuntimeError("pre-grasp position IK did not converge")
    ctrl[:6] = q; ctrl[6] = GRIP_OPEN
    settle(model, data, ctrl, 700, frames, renderer)

    q, ok = ik_target(model, data, tcp, grasp, grasp_mat)
    if not ok:
        raise RuntimeError("horizontal grasp IK did not converge")
    ctrl[:6] = q; ctrl[6] = GRIP_OPEN
    settle(model, data, ctrl, 700, frames, renderer)
    print(f"at-grasp tcp={data.site_xpos[tcp]}  cup={data.xpos[cup]}")

    ctrl[6] = GRIP_CLOSE
    settle(model, data, ctrl, 600, frames, renderer)

    q, ok = ik_target(model, data, tcp, lift, grasp_mat)
    if not ok:
        raise RuntimeError("horizontal lift IK did not converge")
    ctrl[:6] = q
    settle(model, data, ctrl, 900, frames, renderer)
    cup_now = data.xpos[cup].copy()
    rose = cup_now[2] - cup0[2]
    print(f"lift cup={cup_now}  rose {rose*1000:.0f} mm -> "
          f"{'LIFTED' if rose > 0.03 else 'NOT lifted'}")

    Image.fromarray(frames[-1]).save(OUT / "phase6_camera_grasp_final.png")
    gif = [Image.fromarray(f) for f in frames[::2]]
    gif[0].save(OUT / "phase6_camera_grasp.gif", save_all=True,
                append_images=gif[1:], duration=60, loop=0)
    print(f"saved {OUT/'phase6_camera_grasp_final.png'} and .gif")


if __name__ == "__main__":
    main()
