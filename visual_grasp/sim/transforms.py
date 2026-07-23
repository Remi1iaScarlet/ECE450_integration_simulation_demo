"""Phase 4 — coordinate transforms (Route A: ground-truth object pose).

Computes base_T_object for the cup from MuJoCo body poses, reads the world camera
extrinsics/intrinsics, and verifies the whole geometry by projecting the ground-truth
cup center back into the world_cam image -- it should land inside the YOLO bbox.

Frame conventions
-----------------
MuJoCo body/world frames: standard right-handed, z up.
MuJoCo CAMERA frame (data.cam_xmat columns = camera axes in world):
    +x right, +y up, looking along -z  (OpenGL convention).
ROS optical frame (what the real RealSense pipeline assumes):
    +x right, +y down, +z forward.
=> ros_optical = R_flip * mujoco_cam, with R_flip = diag(1, -1, -1).
   This matters for Route B (depth back-projection) in Phase 6; Route A here works
   purely in world/base frames so it is unaffected.

Headless: requires MUJOCO_GL=egl.
    MUJOCO_GL=egl python3 sim/transforms.py
"""
import os
import pathlib
import sys

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import perception  # noqa: E402  (render_rgb, detect)

SCENE = ROOT / "sim" / "models" / "piper" / "scene_grasp.xml"
OUT = ROOT / "sim" / "evidence"
np.set_printoptions(precision=4, suppress=True)


def body_pose(model, data, name):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    T = np.eye(4)
    T[:3, :3] = data.xmat[bid].reshape(3, 3)
    T[:3, 3] = data.xpos[bid]
    return T


def camera_pose(model, data, name):
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    T = np.eye(4)
    T[:3, :3] = data.cam_xmat[cid].reshape(3, 3)
    T[:3, 3] = data.cam_xpos[cid]
    return T, cid


def cam_intrinsics(model, cid, w, h):
    fovy = float(model.cam_fovy[cid])  # vertical FOV, degrees
    f = (h / 2.0) / np.tan(np.deg2rad(fovy) / 2.0)
    return f, f, w / 2.0, h / 2.0  # fx, fy, cx, cy (square pixels, centered)


def project(p_world, T_cam, fx, fy, cx, cy):
    """World point -> pixel in a MuJoCo (OpenGL) camera. Returns (u, v, depth)."""
    R, t = T_cam[:3, :3], T_cam[:3, 3]
    p_c = R.T @ (p_world - t)         # into camera frame
    depth = -p_c[2]                   # camera looks along -z; front => p_c.z < 0
    u = cx + fx * (p_c[0] / depth)
    v = cy - fy * (p_c[1] / depth)    # image v grows downward
    return u, v, depth


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    W, H = 640, 480
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    data.ctrl[:] = model.key_ctrl[home]
    for _ in range(800):
        mujoco.mj_step(model, data)

    # --- Route A: ground-truth transforms ---
    world_T_base = body_pose(model, data, "base_link")
    world_T_cup = body_pose(model, data, "cup")
    base_T_cup = np.linalg.inv(world_T_base) @ world_T_cup
    world_T_cam, cid = camera_pose(model, data, "world_cam")
    base_T_cam = np.linalg.inv(world_T_base) @ world_T_cam

    print("world_T_base (t):", world_T_base[:3, 3])
    print("world_T_cup  (t):", world_T_cup[:3, 3])
    print("world_T_cam  (t):", world_T_cam[:3, 3])
    print("\nbase_T_cup =\n", base_T_cup)
    print("=> cup position in BASE frame:", base_T_cup[:3, 3])
    print("\nbase_T_cam (t):", base_T_cam[:3, 3])

    # --- Verification: project GT cup center into the image, compare to YOLO bbox ---
    fx, fy, cx, cy = cam_intrinsics(model, cid, W, H)
    print(f"\nworld_cam intrinsics: fx=fy={fx:.1f}  cx={cx:.0f} cy={cy:.0f}")
    u, v, depth = project(world_T_cup[:3, 3], world_T_cam, fx, fy, cx, cy)
    print(f"GT cup center -> pixel ({u:.1f}, {v:.1f}), depth={depth:.3f} m")

    rgb = perception.render_rgb(model, data, "world_cam", W, H)
    target_boxes, _, _ = perception.detect(rgb)
    img = Image.fromarray(rgb)
    draw = ImageDraw.Draw(img)
    inside = False
    for cls, conf, (x1, y1, x2, y2) in target_boxes:
        color = (255, 220, 0)
        draw.rectangle([x1, y1, x2, y2], outline=color, width=3)
        draw.text((x1, max(0, y1 - 12)), f"{cls} {conf:.2f}", fill=color)
        bcx, bcy = (x1 + x2) / 2, (y1 + y2) / 2
        draw.line([bcx - 8, bcy, bcx + 8, bcy], fill=color, width=1)
        draw.line([bcx, bcy - 8, bcx, bcy + 8], fill=color, width=1)
        if cls == "cup" and x1 <= u <= x2 and y1 <= v <= y2:
            inside = True
    # GT projected center (red crosshair)
    draw.line([u - 12, v, u + 12, v], fill=(255, 0, 0), width=2)
    draw.line([u, v - 12, u, v + 12], fill=(255, 0, 0), width=2)
    draw.ellipse([u - 4, v - 4, u + 4, v + 4], outline=(255, 0, 0), width=2)
    out = OUT / "phase4_target_proj.png"
    img.save(out)

    print(f"\nGT cup center projects INSIDE YOLO cup bbox: {inside}")
    print(f"saved {out}")
    print("VERDICT:", "TRANSFORM VERIFIED" if inside else "CHECK FRAMES")


if __name__ == "__main__":
    main()
