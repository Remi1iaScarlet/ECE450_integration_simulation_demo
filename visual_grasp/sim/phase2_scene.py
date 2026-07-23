"""Phase 2 — grasp scene + cameras verification.

Loads scene_grasp.xml, lets the bottle/cup settle on the table, renders the fixed
world camera and the eye-in-hand wrist camera, and logs object & camera poses
(useful for the Phase 4 coordinate transform).

Headless: requires MUJOCO_GL=egl.
    MUJOCO_GL=egl python3 sim/phase2_scene.py
"""
import os
import pathlib

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCENE = ROOT / "sim" / "models" / "piper" / "scene_grasp.xml"
OUT = ROOT / "sim" / "evidence"
W, H = 1280, 720

# A backed-off "look" pose (ctrl = j1..j6, gripper) that keeps the eye-in-hand
# camera nearly horizon-level and aimed at the cup.
LOOK_POSE = [0.0, 0.30, 0.0, 0.0, -0.17, 0.0, 0.035]


def render(model, data, cam, w, h):
    r = mujoco.Renderer(model, height=h, width=w)
    r.update_scene(data, camera=cam)
    img = r.render().copy()
    r.close()
    return img


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.vis.global_.offwidth = W
    model.vis.global_.offheight = H
    data = mujoco.MjData(model)

    # Hold arm at home while objects settle on the table.
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home >= 0:
        data.ctrl[:] = model.key_ctrl[home]
    for _ in range(800):
        mujoco.mj_step(model, data)

    # Log object & camera world poses.
    print("=== object world positions (after settle) ===")
    for name in ["bottle", "cup", "table"]:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        print(f"  {name:7s} xpos={np.array2string(data.xpos[bid], precision=4)}")

    print("\n=== camera world poses ===")
    for name in ["world_cam", "wrist_cam"]:
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        pos = data.cam_xpos[cid]
        mat = data.cam_xmat[cid].reshape(3, 3)
        print(f"  {name:9s} pos={np.array2string(pos, precision=4)}  "
              f"forward(-z)={np.array2string(-mat[:, 2], precision=3)}")

    # Render world cam (scene overview) at home.
    Image.fromarray(render(model, data, "world_cam", W, H)).save(OUT / "phase2_world_cam.png")
    print(f"\nsaved {OUT / 'phase2_world_cam.png'}")

    # Move arm to the look pose so the wrist cam points at the table, then render both.
    data.ctrl[:] = LOOK_POSE
    for _ in range(800):
        mujoco.mj_step(model, data)
    Image.fromarray(render(model, data, "wrist_cam", 640, 480)).save(OUT / "phase2_wrist_cam.png")
    Image.fromarray(render(model, data, "world_cam", W, H)).save(OUT / "phase2_world_cam_lookpose.png")
    print(f"saved {OUT / 'phase2_wrist_cam.png'}")
    print(f"saved {OUT / 'phase2_world_cam_lookpose.png'}")


if __name__ == "__main__":
    main()
