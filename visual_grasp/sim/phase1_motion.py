"""Phase 1 — joints-can-move evidence.

Drives the Piper arm to several poses (and opens/closes the gripper) using the
position actuators, renders each settled pose, and writes a 2x2 montage.

Headless: requires MUJOCO_GL=egl.
    MUJOCO_GL=egl python3 sim/phase1_motion.py
"""
import os
import pathlib

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCENE = ROOT / "sim" / "models" / "piper" / "scene.xml"
OUT = ROOT / "sim" / "evidence"

# ctrl = [j1..j6, gripper]; gripper 0.035=open, 0=closed.
POSES = [
    ("home",          [0.0, 1.57, -1.3485, 0.0, 0.0, 0.0, 0.035]),
    ("reach (open)",  [0.6, 1.0, -0.8, 0.0, 0.5, 0.0, 0.035]),
    ("reach (closed)",[0.6, 1.0, -0.8, 0.0, 0.5, 0.0, 0.000]),
    ("twist (closed)",[-0.7, 1.2, -1.0, 0.5, -0.3, 0.6, 0.000]),
]
W, H = 480, 360


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.vis.global_.offwidth = W
    model.vis.global_.offheight = H
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=H, width=W)

    frames = []
    for label, ctrl in POSES:
        data.ctrl[:] = ctrl
        for _ in range(600):  # let the position servo settle
            mujoco.mj_step(model, data)
        # report achieved gripper width and link6 height as a quick sanity log
        gw = data.qpos[6] - data.qpos[7]  # joint7 - joint8 (joint8 mimics, negative)
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
        print(f"{label:16s} link6_z={data.xpos[bid][2]:.3f}  gripper_width={gw*1000:5.1f} mm")
        renderer.update_scene(data)
        img = renderer.render().copy()
        frames.append((label, img))

    # 2x2 montage.
    pad = 6
    canvas = Image.new("RGB", (W * 2 + pad, H * 2 + pad), (20, 20, 20))
    for idx, (label, img) in enumerate(frames):
        r, c = idx // 2, idx % 2
        canvas.paste(Image.fromarray(img), (c * (W + pad), r * (H + pad)))
    out = OUT / "phase1_motion.png"
    canvas.save(out)
    print(f"\nmontage -> {out}")


if __name__ == "__main__":
    main()
