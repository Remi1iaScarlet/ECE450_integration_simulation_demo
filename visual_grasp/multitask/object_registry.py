"""Object registry: perception backend -> structured object instances.

Generalizes phase6_route_b.camera_object_point (which returns only the first cup)
to ALL detected target instances, building one record per object with its 3D
center in the robot base frame. This is the multi-object foundation the task
library selects from.

Backends:
  yolo:   YOLO + RGB-D route, matching the visual_grasp perception path.
  sim_gt: MuJoCo body-pose ground truth, explicitly selected for full simulation
          on machines without YOLO/ultralytics.
"""
import os
import pathlib
import sys

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))

# Reject detections whose ROI is nearer than this (m): that is the gripper in the
# wrist-camera foreground, not a table object.
MIN_OBJECT_DEPTH = 0.20

# For ray-plane container localization, intersect slightly above the table so the
# ray hits the bowl's visual mid-height rather than its far base (parallax). Tuned
# to ~32 mm error on the YCB bowl; the bowl is 16 cm wide so the cup lands inside.
CONTAINER_PLANE_OFFSET = 0.02


def _import_yolo_stack():
    """Import the YOLO/RGB-D helpers only when the yolo backend is selected."""
    try:
        import perception  # noqa: E402
        import phase6_route_b  # noqa: E402
        import transforms  # noqa: E402
    except ModuleNotFoundError as exc:
        if exc.name == "ultralytics":
            raise RuntimeError(
                "YOLO perception backend requires ultralytics. Install it, or "
                "explicitly run with --perception sim_gt / detector.backend: sim_gt "
                "for MuJoCo ground-truth simulation."
            ) from exc
        raise
    return perception, phase6_route_b, transforms


def _body_pose(model, data, name):
    import mujoco
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        return None
    t = np.eye(4)
    t[:3, :3] = data.xmat[bid].reshape(3, 3)
    t[:3, 3] = data.xpos[bid]
    return t


def _camera_pose(model, data, name):
    import mujoco
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    if cid < 0:
        return None, None
    t = np.eye(4)
    t[:3, :3] = data.cam_xmat[cid].reshape(3, 3)
    t[:3, 3] = data.cam_xpos[cid]
    return t, cid


def _project_bbox(model, p_world, world_T_cam, cid, width, height, half_size=12.0):
    if world_T_cam is None or cid is None:
        return [0.0, 0.0, 0.0, 0.0], None
    p_cam = world_T_cam[:3, :3].T @ (p_world - world_T_cam[:3, 3])
    depth = -p_cam[2]
    if depth <= 1e-6:
        return [0.0, 0.0, 0.0, 0.0], p_cam
    f = (height / 2.0) / np.tan(np.deg2rad(float(model.cam_fovy[cid])) / 2.0)
    u = width / 2.0 + f * p_cam[0] / depth
    v = height / 2.0 - f * p_cam[1] / depth
    return [
        round(max(0.0, u - half_size), 1),
        round(max(0.0, v - half_size), 1),
        round(min(float(width), u + half_size), 1),
        round(min(float(height), v + half_size), 1),
    ], p_cam


def _bbox_center_camera(depth, bbox, fx, fy, cx, cy, radius_factor=1.0):
    """ROI back-projection -> 3D center in the camera frame (+ ROI point count).

    Near-surface radius correction as in the validated single-cup Route B: the
    back-projected ROI lands on the surface facing the camera, so push the point
    away from the camera by (radius_factor * bbox-estimated radius) to approximate
    the object center. radius_factor=1.0 is the original full push (Menagerie);
    the wrist-cam median ROI already sits partway into the object, so a smaller
    factor (~0.5, piper_real) avoids over-shooting the far side.
    """
    _, phase6_route_b, _ = _import_yolo_stack()
    pc = phase6_route_b.backproject_roi(depth, bbox, fx, fy, cx, cy)
    if len(pc) < 10:
        return None, len(pc)
    center = phase6_route_b.median_cluster(pc)
    if radius_factor:
        d = -center[2]                              # optical-axis depth
        radius = 0.5 * (bbox[2] - bbox[0]) * d / fx
        center = center + radius_factor * radius * center / np.linalg.norm(center)
    return center, len(pc)


def _bbox_center_on_plane(bbox, world_T_cam, fx, fy, cx, cy, plane_z):
    """Intersect the camera ray through the bbox center with the world plane z=plane_z.

    Robust for a table-resting container (its 3D center is hard to back-project from
    a big concave shape, but it sits on a known table height, so a ray-plane hit at
    the bbox center gives a clean xy)."""
    u = 0.5 * (bbox[0] + bbox[2])
    v = 0.5 * (bbox[1] + bbox[3])
    R, o = world_T_cam[:3, :3], world_T_cam[:3, 3]
    d_world = R @ np.array([(u - cx) / fx, (cy - v) / fy, -1.0])
    if abs(d_world[2]) < 1e-6:
        return None
    t = (plane_z - o[2]) / d_world[2]
    if t <= 0:
        return None
    return o + t * d_world


def detect_objects(world, graspable=(), containers=(), conf=0.3, table_z=0.12,
                   backend="yolo", body_of=None, radius_factor=1.0):
    if backend == "sim_gt":
        return detect_objects_sim_gt(world, graspable, containers, table_z, body_of)
    if backend != "yolo":
        raise ValueError(f"unknown perception backend: {backend}")
    return detect_objects_yolo(world, graspable, containers, conf, table_z, radius_factor)


def detect_objects_yolo(world, graspable=(), containers=(), conf=0.3, table_z=0.12,
                        radius_factor=1.0):
    """Run YOLO on the wrist camera (current pose) and return object records.

    Each record:
        {id, label, confidence, bbox, center_camera, center_base,
         width_estimate, n_roi, graspable, container}

    Graspable objects are localized by ROI depth back-projection (precise 3D for
    grasping). Containers are localized by ray-to-table-plane at the bbox center
    (robust for big concave shapes resting on the known table height).
    """
    objects, _ = collect_yolo_candidates(
        world, graspable, containers, conf, table_z, radius_factor)
    return objects


def collect_yolo_candidates(world, graspable=(), containers=(), min_conf=0.05,
                            table_z=0.12, radius_factor=1.0):
    """Render once and return localized low-confidence candidates plus RGB."""
    perception, _, transforms = _import_yolo_stack()
    model, data, cam = world.model, world.data, world.cam
    w, h = 640, 480
    labels = set(graspable) | set(containers)
    rgb = perception.render_rgb(model, data, cam, w, h)
    candidates = perception.detect_candidates(
        rgb, labels=labels, min_confidence=min_conf)
    if not candidates:
        return [], rgb

    depth = perception.render_depth(model, data, cam, w, h)
    world_T_cam, cid = transforms.camera_pose(model, data, cam)
    fx, fy, cx, cy = transforms.cam_intrinsics(model, cid, w, h)
    world_T_base = transforms.body_pose(model, data, "base_link")
    inv_base = np.linalg.inv(world_T_base)

    objects = []
    counts = {}
    for candidate in candidates:
        label, dconf, bbox = (
            candidate.label, candidate.confidence, candidate.bbox)
        # Depth gate: reject boxes whose ROI is implausibly near -- that is the
        # gripper in the wrist foreground, occasionally misdetected.
        x1, y1, x2, y2 = [int(round(v)) for v in bbox]
        roi = depth[y1:y2, x1:x2]
        if roi.size == 0 or float(np.median(roi)) < MIN_OBJECT_DEPTH:
            continue
        is_container = label in containers
        if is_container:
            p_world = _bbox_center_on_plane(bbox, world_T_cam, fx, fy, cx, cy,
                                            table_z + CONTAINER_PLANE_OFFSET)
            if p_world is None:
                continue
            center_cam, n_roi = None, roi.size
        else:
            center_cam, n_roi = _bbox_center_camera(depth, bbox, fx, fy, cx, cy,
                                                     radius_factor=radius_factor)
            if center_cam is None:
                continue
            p_world = world_T_cam[:3, :3] @ center_cam + world_T_cam[:3, 3]
        center_base = (inv_base @ np.append(p_world, 1.0))[:3]
        idx = counts.get(label, 0)
        counts[label] = idx + 1
        objects.append({
            "id": f"{label}_{idx}",
            "label": label,
            "confidence": float(dconf),
            "bbox": [round(float(v), 1) for v in bbox],
            "center_camera": center_cam,
            "center_base": center_base,
            "n_roi": int(n_roi),
            "graspable": label in graspable,
            "container": is_container,
        })
    return objects, rgb


def detect_objects_sim_gt(world, graspable=(), containers=(), table_z=0.12,
                          body_of=None):
    """Return YOLO/RGB-D-compatible object records from MuJoCo body poses.

    This backend is deliberately opt-in. It is intended for validating the full
    manipulation FSM and place_into behavior in simulation when the detector stack
    is unavailable or known to have a domain gap.
    """
    body_of = body_of or {}
    model, data = world.model, world.data
    world_T_base = _body_pose(model, data, "base_link")
    if world_T_base is None:
        raise RuntimeError("sim_gt perception requires a MuJoCo body named base_link")
    inv_base = np.linalg.inv(world_T_base)
    world_T_cam, cid = _camera_pose(model, data, world.cam)
    labels = list(graspable) + list(containers)

    objects = []
    counts = {}
    for label in labels:
        body_name = body_of.get(label, label)
        body_T = _body_pose(model, data, body_name)
        if body_T is None:
            continue
        is_container = label in containers
        p_world = body_T[:3, 3].copy()
        if is_container:
            p_world[2] = table_z + CONTAINER_PLANE_OFFSET
        center_base = (inv_base @ np.append(p_world, 1.0))[:3]
        bbox, center_camera = _project_bbox(model, p_world, world_T_cam, cid, 640, 480)
        idx = counts.get(label, 0)
        counts[label] = idx + 1
        objects.append({
            "id": f"{label}_{idx}",
            "label": label,
            "confidence": 1.0,
            "bbox": bbox,
            "center_camera": center_camera,
            "center_base": center_base,
            "n_roi": 0,
            "graspable": label in graspable,
            "container": is_container,
            "body": body_name,
            "backend": "sim_gt",
        })
    return objects


def scan(world, scan_poses, graspable=(), containers=(), conf=0.3, table_z=0.12,
         backend="yolo", body_of=None, radius_factor=1.0, motion_config=None):
    """Multi-pose registry: visit each scan pose, detect, and merge the results.

    A cup and a container in one wrist frame suppress each other (YOLO domain gap),
    so detections from several poses are merged into one registry. center_base is
    pose-independent, so the same physical object detected in two poses is deduped
    by base-frame proximity (highest confidence wins).
    """
    if backend == "sim_gt":
        return detect_objects_sim_gt(world, graspable, containers, table_z, body_of)
    if backend != "yolo":
        raise ValueError(f"unknown perception backend: {backend}")
    found = []
    motion_config = motion_config or {}
    continuous = motion_config.get("execution_mode") == "continuous"
    safe = np.asarray(motion_config.get("safe_transit_pose", ()), float)

    def move(ctrl, phase, settle_steps):
        ctrl = np.asarray(ctrl, float)
        if not continuous:
            world.move_to_pose(ctrl)
            return
        world.follow_joint_waypoints(
            (np.asarray(world.data.qpos[:6], float), ctrl[:6]),
            grip=float(ctrl[6]),
            max_joint_velocity_rad_s=float(
                motion_config.get("scan_max_joint_velocity_rad_s", 0.8)),
            max_following_error_rad=float(
                motion_config.get("max_following_error_rad", 0.12)),
            final_settle_steps=int(settle_steps), phase=phase,
            min_table_clearance_m=float(
                motion_config.get("min_table_clearance_m", 0.010)))

    if continuous and safe.size == 7 and not np.allclose(
            world.data.qpos[:6], safe[:6], atol=0.03):
        move(safe, "SAFE_TRANSIT", 0)
    for ctrl in scan_poses:
        move(ctrl, "SCAN", int(motion_config.get("scan_settle_steps", 80)))
        found.extend(detect_objects_yolo(world, graspable, containers, conf, table_z,
                                         radius_factor))
        if continuous and safe.size == 7:
            move(safe, "SAFE_SCAN_EXIT", 0)
    return _merge(found)


def _merge(objects, dist=0.06):
    """Dedupe objects by (label, base-xy proximity); keep the highest-confidence one."""
    merged = []
    for o in sorted(objects, key=lambda x: -x["confidence"]):
        dup = any(
            m["label"] == o["label"]
            and np.linalg.norm(m["center_base"][:2] - o["center_base"][:2]) < dist
            for m in merged
        )
        if not dup:
            merged.append(o)
    counts = {}
    for o in merged:
        idx = counts.get(o["label"], 0)
        counts[o["label"]] = idx + 1
        o["id"] = f"{o['label']}_{idx}"
    return merged


def get_object(objects, label, policy="nearest"):
    """Select one object instance by label using a simple policy.

    Policies: nearest (closest in base xy), highest_confidence, leftmost, rightmost.
    'nearest' breaks near-ties by confidence, so a low-confidence domain-gap
    false positive (e.g. the bottle occasionally read as 'cup') does not win over
    the real, higher-confidence detection at a similar distance.
    """
    candidates = [o for o in objects if o["label"] == label]
    if not candidates:
        return None
    if policy == "highest_confidence":
        return max(candidates, key=lambda o: o["confidence"])
    if policy == "leftmost":
        return min(candidates, key=lambda o: o["bbox"][0])
    if policy == "rightmost":
        return max(candidates, key=lambda o: o["bbox"][2])
    # default: nearest in base xy, with confidence as a tie-breaker (~8 cm bucket,
    # so a closer low-confidence false positive does not beat the real detection)
    return min(candidates, key=lambda o: (round(float(np.linalg.norm(o["center_base"][:2])) / 0.08),
                                          -o["confidence"]))
