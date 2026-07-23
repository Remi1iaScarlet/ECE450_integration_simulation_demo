"""Phase 6 (validation) — Route B: camera-driven 3D localization.

Reproduces the real RealSense pipeline in sim:
  wrist_cam RGB -> YOLO cup bbox -> wrist_cam depth -> ROI back-projection to a
  3D point in the CAMERA frame -> transform to BASE frame via the camera extrinsic
  base_T_camera / link6_T_camera (the simulation "hand-eye calibration", read
  from the model).

Then compares the camera-derived base point against the ground-truth cup pose to
quantify the error. No ground-truth is used to compute the target.

Headless: requires MUJOCO_GL=egl.
    MUJOCO_GL=egl python3 sim/phase6_route_b.py
"""
import os
import pathlib
import sys

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import perception  # noqa: E402
import transforms  # noqa: E402

SCENE = ROOT / "sim" / "models" / "piper" / "scene_grasp.xml"
DEFAULT_CAM = "wrist_cam"

# Eye-in-hand observation pose. Tuned by a headless pose sweep at full settle
# (2026-06-29): base yaw (j1=+0.2) + shoulder (j2=+0.4) lift the wrist and swing
# the cup to the upper-right of the frame, clear of the gripper body that
# otherwise occludes the centre of the wrist view and makes YOLO miss the cup.
# At this settled pose the cup is fully framed and detected (~1700 ROI depth
# points) and Route B back-projection lands within grasp tolerance of the cup.
WRIST_OBSERVE_CTRL = np.array([0.2, 0.4, 0.0, 0.0, 0.0, 0.0, 0.035])
np.set_printoptions(precision=4, suppress=True)


def backproject_roi(depth, bbox, fx, fy, cx, cy, step=3, dmin=0.1, dmax=2.0):
    """ROI pixels -> camera-frame points (MuJoCo cam convention: +x right,+y up,-z fwd)."""
    x1, y1, x2, y2 = [int(round(v)) for v in bbox]
    us = np.arange(x1, x2, step)
    vs = np.arange(y1, y2, step)
    uu, vv = np.meshgrid(us, vs)
    d = depth[vv, uu]
    m = (d > dmin) & (d < dmax)
    uu, vv, d = uu[m], vv[m], d[m]
    # inverse of transforms.project(): p_c = [(u-cx)d/fx, (cy-v)d/fy, -d]
    pc = np.stack([(uu - cx) * d / fx, (cy - vv) * d / fy, -d], axis=1)
    return pc


def median_cluster(points, thresh=0.05):
    """Robust center: keep points within thresh of the per-axis median, then mean."""
    med = np.median(points, axis=0)
    keep = np.linalg.norm(points - med, axis=1) <= thresh
    return points[keep].mean(axis=0)


def move_to_observation_pose(model, data, steps=800):
    """Move the arm to the backed-off wrist-camera observation pose."""
    data.ctrl[:] = WRIST_OBSERVE_CTRL
    for _ in range(steps):
        mujoco.mj_step(model, data)


def camera_object_point(model, data, cam=DEFAULT_CAM, W=640, H=480, radius_correct=True):
    """Full Route B: returns (base_point, n_roi_points, bbox) or (None, 0, None).

    The back-projected ROI lands on the cup's NEAR surface. If radius_correct, we
    estimate the cup radius from the bbox width and depth (camera-only) and push the
    point away from the camera by that radius to approximate the cup center.
    """
    rgb = perception.render_rgb(model, data, cam, W, H)
    boxes, _, _ = perception.detect(rgb)
    cup = [b for b in boxes if b[0] == "cup"]
    if not cup:
        return None, 0, None
    bbox = cup[0][2]
    depth = perception.render_depth(model, data, cam, W, H)
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam)
    fx, fy, cx, cy = transforms.cam_intrinsics(model, cid, W, H)
    pc = backproject_roi(depth, bbox, fx, fy, cx, cy)
    if len(pc) < 10:
        return None, len(pc), bbox
    center_cam = median_cluster(pc)
    if radius_correct:
        d = -center_cam[2]                                   # optical-axis depth
        radius = 0.5 * (bbox[2] - bbox[0]) * d / fx          # est. radius from bbox width
        center_cam = center_cam + radius * center_cam / np.linalg.norm(center_cam)
    # camera -> world -> base
    world_T_cam, _ = transforms.camera_pose(model, data, cam)
    world_T_base = transforms.body_pose(model, data, "base_link")
    p_world = world_T_cam[:3, :3] @ center_cam + world_T_cam[:3, 3]
    p_base = np.linalg.inv(world_T_base) @ np.append(p_world, 1.0)
    return p_base[:3], len(pc), bbox


def main():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    data.ctrl[:] = model.key_ctrl[home]
    for _ in range(800):
        mujoco.mj_step(model, data)
    move_to_observation_pose(model, data)

    # The simulation "hand-eye calibration": extrinsic read straight from the model.
    world_T_cam, _ = transforms.camera_pose(model, data, DEFAULT_CAM)
    world_T_base = transforms.body_pose(model, data, "base_link")
    world_T_link6 = transforms.body_pose(model, data, "link6")
    base_T_cam = np.linalg.inv(world_T_base) @ world_T_cam
    link6_T_cam = np.linalg.inv(world_T_link6) @ world_T_cam
    print(f"=== sim hand-eye calibration for {DEFAULT_CAM} (read from model) ===")
    print("base_T_camera =")
    print(base_T_cam)
    print("\nlink6_T_camera =")
    print(link6_T_cam)

    p_base, n, bbox = camera_object_point(model, data)
    if p_base is None:
        raise SystemExit(
            f"no cup detected by {DEFAULT_CAM} (bbox={bbox}, {n} ROI points); "
            "cannot localize. Run sim/perception.py to inspect the wrist view / YOLO output."
        )
    cup = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cup")
    gt = data.xpos[cup].copy()
    print(f"\nYOLO cup bbox: {[round(v,1) for v in bbox]}  ({n} ROI depth points)")
    print(f"camera-derived cup point (BASE): {p_base}")
    print(f"ground-truth cup center (BASE):  {gt}")
    err = p_base - gt
    print(f"error vector: {err}  |err|={np.linalg.norm(err)*1000:.0f} mm "
          f"(xy={np.linalg.norm(err[:2])*1000:.0f} mm, z={err[2]*1000:.0f} mm)")
    print("\nnote: camera sees the cup's NEAR surface, so the point is biased toward "
          "the camera by ~radius; xy error is what matters for the pinch grasp.")


if __name__ == "__main__":
    main()
