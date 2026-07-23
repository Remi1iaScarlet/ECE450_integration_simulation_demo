"""Phase 3 — MuJoCo wrist camera -> YOLO detection.

Moves the arm to the wrist-camera observation pose, renders RGB from wrist_cam,
and runs the same YOLO model / class filter as the real pipeline
(realsense_yolo_pc_roi.py: bottle/cup, conf>0.3). Logs ALL detections too, so any
sim-to-real domain gap (e.g. the bottle detected as 'vase') is visible.

Headless: requires MUJOCO_GL=egl.
    MUJOCO_GL=egl python3 sim/perception.py
"""
import os
import pathlib
from dataclasses import dataclass

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402
from ultralytics import YOLO  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCENE = ROOT / "sim" / "models" / "piper" / "scene_grasp.xml"
WEIGHTS = ROOT / "yolo11n.pt"
OUT = ROOT / "sim" / "evidence"
CAM = "wrist_cam"
# Keep in sync with phase6_route_b.WRIST_OBSERVE_CTRL (sweep-tuned so the cup
# clears the gripper in the wrist view; see that file for the rationale).
WRIST_OBSERVE_CTRL = np.array([0.2, 0.4, 0.0, 0.0, 0.0, 0.0, 0.035])

TARGET_CLASSES = {"bottle", "cup"}
CONF = 0.3

_model = None
_target_class_ids = None


@dataclass(frozen=True)
class DetectionCandidate:
    label: str
    confidence: float
    bbox: tuple[float, float, float, float]


def get_model():
    global _model
    if _model is None:
        _model = YOLO(str(WEIGHTS))
    return _model


def target_class_ids(model):
    """COCO class ids that the visual grasp pipeline is allowed to detect."""
    global _target_class_ids
    if _target_class_ids is None:
        _target_class_ids = [
            class_id for class_id, name in model.names.items()
            if name in TARGET_CLASSES
        ]
    return _target_class_ids


def detect_candidates(rgb, labels=None, min_confidence=0.05):
    """Return low-threshold candidates for offline calibration and filtering."""
    model = get_model()
    allowed = set(labels) if labels is not None else TARGET_CLASSES
    class_ids = [cid for cid, name in model.names.items() if name in allowed]
    result = model.predict(
        rgb, verbose=False, classes=class_ids, conf=float(min_confidence))[0]
    candidates = []
    for box in result.boxes:
        label = model.names[int(box.cls)]
        if label not in allowed:
            continue
        candidates.append(DetectionCandidate(
            label=label, confidence=float(box.conf),
            bbox=tuple(float(v) for v in box.xyxy[0].tolist())))
    return candidates


def render_rgb(model, data, cam, w=640, h=480):
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, w)
    model.vis.global_.offheight = max(model.vis.global_.offheight, h)
    r = mujoco.Renderer(model, height=h, width=w)
    r.update_scene(data, camera=cam)
    img = r.render().copy()
    r.close()
    return img  # RGB uint8 (H,W,3)


def render_depth(model, data, cam, w=640, h=480):
    """Metric depth (HxW float32, meters) from a MuJoCo camera."""
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, w)
    model.vis.global_.offheight = max(model.vis.global_.offheight, h)
    r = mujoco.Renderer(model, height=h, width=w)
    r.enable_depth_rendering()
    r.update_scene(data, camera=cam)
    depth = r.render().copy()
    r.close()
    return depth


def detect(rgb, conf=CONF, labels=None):
    """Run YOLO on an RGB image. Returns (target_boxes, all_dets, annotated_rgb).

    `labels` is the set of class names allowed as targets (default bottle/cup, the
    original pipeline). The multitask registry passes a wider set (e.g. adding
    container classes) so the same detection path can find cups and containers.
    """
    model = get_model()
    allowed = set(labels) if labels is not None else TARGET_CLASSES
    class_ids = [cid for cid, name in model.names.items() if name in allowed]
    results = model.predict(rgb, verbose=False, classes=class_ids, conf=conf)
    res = results[0]
    names = model.names
    all_dets, target_boxes = [], []
    for box in res.boxes:
        cls = names[int(box.cls)]
        c = float(box.conf)
        xyxy = box.xyxy[0].tolist()
        all_dets.append((cls, c, xyxy))
        if cls in allowed:
            target_boxes.append((cls, c, xyxy))
    annotated = res.plot()  # input was RGB, so plot() returns RGB
    return target_boxes, all_dets, annotated


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home >= 0:
        data.ctrl[:] = model.key_ctrl[home]
    for _ in range(800):
        mujoco.mj_step(model, data)
    data.ctrl[:] = WRIST_OBSERVE_CTRL
    for _ in range(800):
        mujoco.mj_step(model, data)

    rgb = render_rgb(model, data, CAM, 640, 480)
    Image.fromarray(rgb).save(OUT / "phase3_wrist_rgb.png")

    target_boxes, all_dets, annotated = detect(rgb)
    Image.fromarray(annotated).save(OUT / "phase3_yolo_wrist.png")

    print(f"=== target detections only ({sorted(TARGET_CLASSES)}, conf>{CONF}) on {CAM} ===")
    if not all_dets:
        print("  (none)")
    for cls, c, xyxy in sorted(all_dets, key=lambda d: -d[1]):
        xy = [round(v, 1) for v in xyxy]
        print(f"  {cls:12s} conf={c:.3f}  bbox={xy}")

    print(f"\n=== TARGET detections (bottle/cup, conf>{CONF}) ===")
    if not target_boxes:
        print("  (none) -- domain gap likely; see ALL detections above")
    for cls, c, xyxy in target_boxes:
        print(f"  {cls:12s} conf={c:.3f}  bbox={[round(v,1) for v in xyxy]}")

    print(f"\nsaved {OUT/'phase3_wrist_rgb.png'}  and  {OUT/'phase3_yolo_wrist.png'}")


if __name__ == "__main__":
    main()
