"""Phase 0 — Piper MuJoCo model load verification.

Loads the Menagerie agilex_piper scene headless, prints the model structure
(joints / actuators / ranges / keyframes) and renders the `home` keyframe to a PNG.

Headless: requires `MUJOCO_GL=egl` on this box (no X11). See devlog/DEVLOG.md.
Usage:
    MUJOCO_GL=egl python3 sim/check_model.py
"""
import os
import pathlib

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")  # headless GPU offscreen render
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
SCENE = HERE / "models" / "piper" / "scene.xml"
OUT = HERE / "evidence"


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    print(f"model loaded: {SCENE}")
    print(f"  nq={model.nq}  nv={model.nv}  nu={model.nu}  nbody={model.nbody}")

    print("\njoints (name, type, range):")
    jtype = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        lo, hi = model.jnt_range[i]
        limited = bool(model.jnt_limited[i])
        rng = f"[{lo:.4f}, {hi:.4f}]" if limited else "(unlimited)"
        print(f"  {i}: {name:10s} {jtype[model.jnt_type[i]]:6s} {rng}")

    print("\nactuators (name -> joint):")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        jid = model.actuator_trnid[i, 0]
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        print(f"  {i}: {name:10s} -> {jname}")

    print("\nkeyframes:")
    for i in range(model.nkey):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, i)
        print(f"  {i}: {name}  qpos={np.array2string(model.key_qpos[i], precision=3)}")

    # Reset to the 'home' keyframe and settle.
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key)
        data.ctrl[:] = model.key_ctrl[key]
    mujoco.mj_forward(model, data)
    for _ in range(200):
        mujoco.mj_step(model, data)

    # Report a few body world positions (sanity / for Phase 1 FK alignment).
    print("\nbody world positions at 'home' (after settle):")
    for b in ["base_link", "link6", "link7", "link8"]:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if bid >= 0:
            p = data.xpos[bid]
            print(f"  {b:10s} xpos={np.array2string(p, precision=4)}")

    # Render. The Menagerie model's default offscreen framebuffer is 640x480;
    # enlarge it so we can render at higher resolution.
    model.vis.global_.offwidth = 960
    model.vis.global_.offheight = 720
    renderer = mujoco.Renderer(model, height=720, width=960)
    renderer.update_scene(data)
    img = renderer.render()
    out_path = OUT / "phase0_piper_home.png"
    Image.fromarray(img).save(out_path)
    print(f"\nrendered -> {out_path}  shape={img.shape}")


if __name__ == "__main__":
    main()
