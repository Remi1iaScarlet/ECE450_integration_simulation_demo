# Demo Commands

This file collects runnable demo commands for the current `visual_grasp` project.

Run all commands from the project root:

```bash
cd "/Users/wangrh/undergrad/大四/第三学期/毕设/visual_grasp"
```

The tested Python environment on this machine is:

```bash
/Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python
```

On macOS, MuJoCo viewer demos should use:

```bash
/Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/mjpython
```

## 1. Natural Language Parsing Only

These commands only parse natural-language commands into structured tasks. They do not run MuJoCo.

```bash
python3 -m multitask.run_nl "把杯子放到碗里" --dry-run
```

```bash
python3 -m multitask.run_nl "pick cup" --dry-run
```

```bash
python3 -m multitask.run_nl "clear table into bowl" --dry-run
```

Expected output includes a parsed JSON task, for example:

```text
task = place_into
source_label = cup
container_label = bowl
```

## 2. Full Natural Language Simulation

This is the main graduation-design demo path:

natural language -> keyword extraction -> task selection -> simulated perception -> IK control -> robot execution.

Use `--perception sim_gt` for a stable full-simulation demo. This explicitly uses MuJoCo object poses as the perception backend. The default backend remains YOLO/RGB-D.

```bash
env MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python -m multitask.run_nl "把杯子放到碗里" --perception sim_gt --no-evidence
```

With evidence GIF/PNG output enabled:

```bash
env MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python -m multitask.run_nl "把杯子放到碗里" --perception sim_gt
```

Evidence is saved under:

```text
multitask/evidence/
```

## 3. Structured Multitask Executor

Run the same task without natural language parsing.

Pick up the cup:

```bash
env MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python -m multitask.executor --task pick --target cup --perception sim_gt --no-evidence
```

Place the cup into the bowl:

```bash
env MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python -m multitask.executor --task place_into --target cup --container bowl --perception sim_gt --no-evidence
```

Place the cup at a fixed base-frame coordinate:

```bash
env MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python -m multitask.executor --task place_at --target cup --xyz 0.40,0.12,0.17 --perception sim_gt --no-evidence
```

Clear table into the bowl:

```bash
env MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python -m multitask.executor --task clear_table --labels cup,bottle --container bowl --perception sim_gt --no-evidence
```

## 4. MuJoCo Viewer Demo

This opens the interactive MuJoCo viewer. It is the existing camera-driven single-cup grasp demo.

```bash
MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/mjpython sim/grasp_sim_viewer.py
```

Keep the final pose visible for 10 seconds:

```bash
MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/mjpython sim/grasp_sim_viewer.py --hold 10
```

Use free camera view:

```bash
MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/mjpython sim/grasp_sim_viewer.py --view free
```

Run at 2x speed and hold the final pose:

```bash
MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/mjpython sim/grasp_sim_viewer.py --speed 2 --hold 10
```

## 5. Headless Single-Stage Simulation Checks

Check the MuJoCo Piper model and save a rendered image:

```bash
env MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python sim/check_model.py
```

Run the original single-object grasp simulation:

```bash
env MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python sim/grasp_sim.py
```

Run wrist-camera RGB-D localization check:

```bash
env MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python sim/phase6_route_b.py
```

Run MuJoCo wrist camera + YOLO detection:

```bash
env MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/python sim/perception.py
```

## 6. YOLO / Hardware-Related Tests

Standalone YOLO image test:

```bash
python3 test_yolo.py
```

RealSense camera test. This requires a connected RealSense camera:

```bash
python3 test_realsense.py
```

ROS point-cloud test. This requires ROS and a connected RealSense camera:

```bash
python3 test_depth_2_pointcloud.py
```

## Notes

`sim_gt` is for stable simulation validation. It is useful for proving the task execution chain works, especially when YOLO has simulation domain-gap issues.

The default perception backend is still `yolo`, which is the intended visual-grasp perception route.

Use `MUJOCO_GL=glfw` on macOS. On Linux headless machines, `MUJOCO_GL=egl` may be more appropriate.
