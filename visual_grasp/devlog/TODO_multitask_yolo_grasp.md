# TODO: Multi-task YOLO-based Grasping Pipeline

## 1. Goal

This document describes the next-step TODO for extending the current `visual_grasp`
baseline from a single-object visual grasp demo into a small multi-task manipulation
pipeline.

The current baseline already supports:

```text
RGB-D camera
-> YOLO object detection
-> ROI depth back-projection
-> 3D object center estimation
-> camera/base coordinate transform
-> IK solving
-> arm motion + gripper close
```

The proposed next step is **not** to add natural-language input yet. Instead, this
TODO focuses only on the downstream execution layer:

```text
YOLO/RGB-D perception
-> object registry
-> grasp/place primitives
-> task library
-> multi-step task executor
```

The expected result is a reusable task layer that can later be called by an LLM,
GUI, CLI, OpenHarmony bridge, or test script.

## 2. Scope

### In Scope

- Support multiple detected object instances instead of only the first YOLO bbox.
- Build an object registry from YOLO detections and RGB-D depth.
- Define a small task library based on reliable primitives.
- Implement simple multi-step tasks such as:
  - `pick(object_label)`
  - `place_at(object_label, xyz)`
  - `place_on(source_label, target_label)`
  - `place_into(source_label, container_label)`
  - `clear_table(object_labels, container_label)`
- Keep execution rule-based and testable.
- Reuse the existing `visual_grasp` perception and IK/control code as much as possible.

### Out of Scope for This TODO

- Natural-language input.
- LLM planning.
- Vision-language-action policy training.
- Complex grasp pose learning.
- General-purpose motion planning in cluttered scenes.
- Full semantic scene understanding.

These can be added later after the primitive task layer is stable.

## 3. Recommended Architecture: Plan A

Plan A is the fastest practical extension of the current codebase:

```text
YOLO bbox or mask
-> RGB-D ROI point extraction
-> robust 3D center estimation
-> fixed geometric grasp pose
-> IK
-> primitive task executor
```

This approach is suitable for a graduation-design demo because it keeps the
system explainable and directly compatible with the current `visual_grasp`
pipeline.

### Why This Approach

- It reuses the existing YOLO + RGB-D + IK pipeline.
- It avoids training a new grasp network at the beginning.
- It keeps the robot control layer deterministic and easy to debug.
- It creates a clean interface for future natural-language parsing.
- It supports meaningful demos such as "pick cup", "put cup into box", and
  "clear objects into bin".

### Limitations

- Grasp pose quality is heuristic.
- Works best for simple objects such as cups, bottles, blocks, small boxes, and pens.
- Dense clutter and occlusion will reduce reliability.
- Containers need simple geometry assumptions, such as using their detected center
  as the drop target.

## 4. Proposed Module Structure

Suggested new files:

```text
visual_grasp/
├── multitask/
│   ├── __init__.py
│   ├── object_registry.py      # YOLO/RGB-D detections -> structured objects
│   ├── primitives.py           # pick/place/open/close/move_above
│   ├── task_library.py         # pick, place_on, place_into, clear_table
│   ├── executor.py             # finite-state execution and retry logic
│   └── config.yaml             # object labels, task parameters, offsets
```

The existing scripts should remain usable:

```text
realsense_yolo_pc_roi.py
grasp_action.py
sim/perception.py
sim/grasp_sim_camera.py
```

The new `multitask/` layer should wrap existing logic instead of rewriting the
full perception/control stack.

## 5. Object Registry

The current code mostly assumes one target object. For multi-task execution, each
YOLO detection should become an object instance.

Example object record:

```python
{
    "id": "cup_0",
    "label": "cup",
    "confidence": 0.82,
    "bbox": [x1, y1, x2, y2],
    "center_camera": [x, y, z],
    "center_base": [x, y, z],
    "size_estimate": [dx, dy, dz],
    "graspable": True,
    "container": False,
}
```

Minimum functions:

```python
def detect_objects(rgb, depth, intrinsics) -> list[dict]:
    """Run YOLO and build object records."""

def get_object(objects, label: str, policy: str = "nearest") -> dict | None:
    """Select one object instance by label."""

def update_registry() -> list[dict]:
    """Refresh current scene objects."""
```

Selection policies can start simple:

- `nearest`: choose object closest to the arm/camera.
- `highest_confidence`: choose the most confident detection.
- `leftmost` / `rightmost`: useful for deterministic tests.

## 6. Task Primitives

Primitives should be small, reusable, and easy to test.

Recommended primitive set:

```python
def detect(label: str | None = None) -> list[dict]:
    pass

def move_above(point_base, z_offset=0.10) -> bool:
    pass

def move_to(point_base) -> bool:
    pass

def open_gripper() -> bool:
    pass

def close_gripper() -> bool:
    pass

def lift(z_offset=0.10) -> bool:
    pass

def pick_object(obj: dict) -> bool:
    pass

def release_at(point_base) -> bool:
    pass
```

The first implementation can use the current fixed grasp pose strategy:

```text
object center in base frame
-> pre-grasp point above / beside object
-> grasp point
-> close gripper
-> lift
```

## 7. Task Library

The task library composes primitives into useful behaviors.

### pick

```text
detect source
-> estimate grasp point
-> move above source
-> move to grasp point
-> close gripper
-> lift
```

Example:

```python
pick("cup")
```

### place_at

```text
move above target xyz
-> move to target xyz
-> open gripper
-> retreat
```

Example:

```python
place_at("cup", [0.35, -0.10, 0.20])
```

### place_on

```text
pick source
-> detect target
-> target point = target center + top offset
-> release at target point
```

Example:

```python
place_on("cup", "box")
```

### place_into

```text
pick source
-> detect container
-> target point = container center + opening height offset
-> release at target point
```

Example:

```python
place_into("pen", "bucket")
```

First version can assume the container opening is near the detected center. For a
demo, this is acceptable if the container is open, upright, and clearly visible.

### clear_table

```text
for each object label:
    pick object
    place_into object into bin
```

Example:

```python
clear_table(["cup", "bottle", "pen"], "box")
```

## 8. Finite-state Executor

Use a finite-state machine first. A behavior tree can be added later if task
composition becomes complex.

Recommended state flow:

```text
IDLE
-> DETECT_SOURCE
-> PLAN_PICK
-> EXECUTE_PICK
-> VERIFY_GRASP
-> DETECT_TARGET
-> PLAN_PLACE
-> EXECUTE_PLACE
-> VERIFY_DONE
-> DONE / FAILED
```

Each state should return:

```python
{
    "success": bool,
    "message": str,
    "data": dict,
}
```

This makes debugging and future UI/LLM integration easier.

## 9. Configuration

Avoid hard-coding object classes inside the task logic.

Example `config.yaml`:

```yaml
detector:
  confidence_threshold: 0.3

objects:
  graspable:
    - cup
    - bottle
    - pen
    - block

  containers:
    - bucket
    - bowl
    - box

grasp:
  pregrasp_z_offset: 0.10
  lift_z_offset: 0.12
  default_gripper_open: 0.10
  default_gripper_close: 0.06

place:
  container_release_z_offset: 0.08
  surface_release_z_offset: 0.04
```

For COCO YOLO weights, labels such as `cup` and `bottle` are available. Labels
such as `pen`, `bucket`, and custom containers may require fine-tuning a YOLO
model on a small custom dataset.

## 10. Verification Plan

### Stage 1: Single-object Regression  ✅ DONE (2026-06-29, headless MUJOCO_GL=egl)

Goal: make sure the new wrapper does not break the existing grasp demo.

- [x] Detect one `cup`.  (object_registry.detect_objects -> cup_0, conf 0.39, 1702 ROI)
- [x] Build one object registry entry.
- [x] Run `pick("cup")`.  (`python -m multitask.executor --target cup`)
- [x] Verify the object is lifted.  (FSM VERIFY_GRASP: rose 134 mm; evidence multitask/evidence/pick_cup.gif)

### Stage 2: Two-object Pick and Place  ✅ (2026-06-30, container = bowl not box)

Goal: validate source and target selection.

- [x] Detect `cup` and container (bowl; YCB textured mesh).
- [x] Run `place_into("cup", bowl)`.  (cup placed 34 mm from bowl center)
- [x] Verify cup is released near container center.

### Stage 3: Multi-object Loop  ✅ (2026-06-30)

Goal: validate task iteration.

- [x] Detect multiple objects.  (cup + bottle + bowl, all YOLO-detected via textured meshes)
- [x] Run `clear_table(["cup", "bottle"], bowl)`.  (cup 45 mm, bottle 11 mm into bowl; evidence clear_table_cup_final.png)
- [x] Log each attempt result.  (per-object structured log)

### Stage 4: Real Robot Check

Goal: transfer the stable task layer to the Piper + RealSense setup.

- [ ] Confirm camera calibration.
- [ ] Confirm target point in `base_link`.
- [ ] Run at low speed.
- [ ] Add emergency stop procedure.

## 11. Suggested Milestones

### M1: Object Registry  ✅ DONE (2026-06-29) — multitask/object_registry.py

- [x] Refactor YOLO result parsing to return all target objects.
- [x] Compute `center_camera` for each object.
- [x] Transform each object into `center_base`.
- [x] Save debug images and object logs.  (JSON state log + multitask/evidence/*.gif)

### M2: Primitive API  ✅ DONE (2026-06-30), multitask/primitives.py

- [x] Implement `pick_object(obj)`.  (plan_horizontal_grasp + execute_grasp)
- [x] Implement `release_at(point_base)`.  (implemented; verified in M3 place)
- [x] Implement gripper open/close wrappers.
- [x] Add basic retry and error messages.  (structured {success,message,data}; bounded grasp retry as an FSM back-edge VERIFY_GRASP->EXECUTE_PICK, grasp.max_retries in config -- verified: normal pick no retry, forced failure re-attempts 3x then bounded-fails)

### M3: Task Library  ✅ DONE (2026-06-30, all vision-driven), multitask/{executor,task_library}.py

- [x] Implement `pick(label)`.  (verified, Stage 1; lifts ~112 mm)
- [x] Implement `place_at(label, point_base)`.  (verified: cup placed ~25 mm from target)
- [x] Implement `place_into(source_label, container_label)`.  (verified VISION end-to-end: textured YCB 024_bowl detected as 'bowl' 0.91, cup placed in bowl 34 mm; evidence place_into_cup_final.png. Untextured synthetic bowl was unrecognizable -- texture was the fix. Also supports explicit container_xyz.)
- [x] Implement `clear_table(labels, container_label)`.  (verified: clear_table([cup]) -> 51 mm)

### M4: Demo Script  ✅ DONE (2026-06-30)

- [x] Add CLI script for testing tasks.  (`python -m multitask.executor --task {pick,place_at,place_into,clear_table} ...`)
- [x] Example:

```bash
python -m multitask.executor --task place_into --target cup        # vision bowl
python -m multitask.executor --task clear_table --labels cup,bottle
```

### M5: Bridge-ready Interface  ✅ DONE (2026-06-30)

- [x] Make task calls return structured JSON.  (executor + task_library return {task, success, states, data})
- [x] Add logs for perception, planning, IK, and execution.  (per-state log in the result)
- [x] Keep API ready for future natural-language parser.  (DONE by runhanw, merged to main 2026-06-30: `multitask/nl.py` rule-based CN/EN parser -> ParsedTask -> executor (`run_nl.py` CLI); `multitask/bridge.py` bridge-ready JSON entrypoint schema `visual_grasp.bridge.v1`, --dry-run without MuJoCo, backend sim_mujoco vs real_piper (stubbed); `sim_gt` perception backend for YOLO-free machines. My earlier command_parser.py was superseded by nl.py and removed.)

## 12. Future Extensions

After Plan A is stable, possible upgrades include:

- YOLO segmentation instead of bbox-only detection.
- Plane segmentation to remove table points.
- PCA-based object orientation estimation.
- Multiple grasp strategies: top-down, side-grasp, center pinch.
- Behavior tree executor for complex tasks and retry logic.
- Integration with MoveIt Task Constructor or ROS2 actions.
- Learned grasp pose models such as GGCNN, Dex-Net, GraspNet, or AnyGrasp.
- Natural-language parser that maps commands to task-library calls.

## 13. Design Principle

The LLM, GUI, or OpenHarmony frontend should not directly control joints.

The intended long-term architecture is:

```text
high-level command
-> validated task-library call
-> YOLO/RGB-D perception
-> IK/control primitive
-> robot execution
```

This keeps the system explainable, testable, and safer than direct language-to-joint
control.
