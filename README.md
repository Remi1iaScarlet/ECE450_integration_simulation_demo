# ECE450 Integration Simulation Demo

One command, from natural language to a MuJoCo grasp: **"put the cup in the
bowl"** &rarr; LLM intent &rarr; (OpenHarmony/ROS2/ROS1 hop, mocked here)
&rarr; visual_grasp's MuJoCo pipeline &rarr; the cup ends up in the bowl.

This repo combines three ECE450 capstone teammates' repositories. It does
not replace any of them -- see [CREDITS.md](CREDITS.md) for exact source
commits and what was changed.

## Architecture

```text
 text command
      |
      v
 llm/  (Mingrui Li -- robot-command-demo)
   FastAPI + LLM intent mapper -> validated {intent, parameters} JSON
      |
      v
 integration/oh_bridge_mock.py   <-- stand-in, see "Scope and honesty" below
   (real version: oh_bridge/ -- NikKk0o -- OpenHarmony QEMU -> ROS2 Humble
    -> TCP -> ROS1 Noetic -> visual_grasp ROS1 gateway)
      |
      v
 visual_grasp/multitask/bridge.py  (chuhan + runhanw + team)
   backend=sim_mujoco -> TaskExecutor -> MuJoCo
      |
      v
 cup ends up in the bowl (or a clear error if it doesn't)
```

## Scope and honesty

The **OpenHarmony/ROS2/ROS1 hop is mocked**, not run for real, in this repo.
`integration/oh_bridge_mock.py` is a plain Python function that reproduces
the JSON shape (`visual_grasp.bridge.v1`) the real chain produces, so it's a
drop-in replacement point, not a redesign.

Why it's mocked instead of run for real:

- This demo was assembled on a machine with no KVM (`/dev/kvm` absent), no
  `docker`, no `ros2`, and no `hdc`. OpenHarmony's QEMU emulator needs
  hardware-accelerated virtualization to be usable interactively; without it,
  boot and DDS communication are impractically slow.
- The official lightweight `device_qemu` prebuilt images are LiteOS-kernel
  "small system" demos (microcontroller targets) -- they cannot run
  ROS2/CycloneDDS, so they can't stand in for the real thing either.
- The real OpenHarmony-in-the-loop setup (`oh_bridge/docs/FULL_LAUNCH_AND_CONNECTION_GUIDE.md`)
  is a from-source OpenHarmony build + custom QEMU scripts that currently
  only run on Tim's own machine. Even the teammate who owns the OpenHarmony
  Robot Sim work (`GC1-ZhuangHanyang-OpenHarmony-with-Robotic-Arm`) documents
  that proving control genuinely runs *inside* OpenHarmony OS is still an
  open item for the whole team, not something specific to this repo.

So: the LLM leg and the MuJoCo leg in this repo are real and actually run.
The middle hop is a faithful-shaped placeholder. Swapping it for the real
ROS2/ROS1 chain is future work, to be done on a machine that already has
OpenHarmony QEMU working (Tim's), not rebuilt from scratch here.

## Quick start

### Offline (no API key, no server -- what was used to test this repo)

```bash
MUJOCO_GL=egl python3 integration/run_demo.py --llm-json \
  '{"intent": "place_into", "parameters": {"source_label": "cup", "container_label": "bowl"}}'
```

(`run_demo.py` resolves all paths from its own file location, so it can be
run from any working directory.)

This skips the LLM call entirely and feeds an already-mapped intent straight
into the bridge, so it needs no `ANTHROPIC_API_KEY`/`OPENAI_API_KEY`. It still
runs through `llm/backend/validator.py`'s whitelist, so a malformed or
unsupported intent is rejected the same way a real LLM output would be.

Add `--no-evidence` to skip saving a GIF/PNG of the run; drop it to get one
under `visual_grasp/multitask/evidence/bridge_<task>_<object>.gif`, or add
`--mp4` for a full-resolution MP4 in the same folder instead (better for
filming off a screen than the downscaled GIF).

### Live interactive viewer (needs a machine with a real display)

`run_demo.py` renders offscreen (`MUJOCO_GL=egl`) and only produces a
GIF/MP4 after the fact -- there's no window to watch live. If you're on a
machine with an actual display (not this headless build machine),
`integration/live_viewer_demo.py` opens a real MuJoCo window you can orbit
while the task runs:

```bash
MUJOCO_GL=glfw python3 integration/live_viewer_demo.py --command "put the cup in the bowl"
```

Same `--command` / `--llm-json` / `--from-queue` input modes as `run_demo.py`.
On macOS this needs `mjpython` instead of `python3` (MuJoCo passive-viewer
main-thread requirement, same as `visual_grasp/multitask/grasp_viewer.py`);
on Linux, plain `python3` with a working X/Wayland session is enough.

**Caveat**: this was written and sanity-checked on a machine with no
display -- confirmed everything up through the LLM call, validation, and
world construction runs cleanly, and fails exactly (and only) at
`mujoco.viewer.launch_passive()` with a GLFW "no DISPLAY" error, which is
the expected failure mode headless. The window itself opening and rendering
correctly has not been visually confirmed; if it misbehaves on your display
machine, the error message + which line it's on is what to report back.

### Real LLM call, no server

```bash
cd llm && pip install -r requirements.txt && cp .env.example .env
# edit .env: set LLM_API_KEY
cd ..
MUJOCO_GL=egl python3 integration/run_demo.py --command "put the cup in the bowl"
```

### Full flow with the web UI + human approval (closest to real usage)

```bash
cd llm && uvicorn backend.main:app --host 127.0.0.1 --port 8000 &
# open http://127.0.0.1:8000/app, submit "put the cup in the bowl"
# open http://127.0.0.1:8000/queue, approve it
cd ..
MUJOCO_GL=egl python3 integration/run_demo.py --from-queue
```

`run_demo.py --from-queue` marks the queue item `executed` or `failed` on the
LLM service when it's done, same as a real robot worker would.

### Other tasks

```bash
--llm-json '{"intent": "pick", "parameters": {"source_label": "bottle"}}'
--llm-json '{"intent": "clear_table", "parameters": {"labels": ["cup", "bottle"], "container_label": "bowl"}}'
```

## Repo layout

```text
llm/            robot-command-demo (Mingrui) -- patched, see CREDITS.md
oh_bridge/      capstone-openharmony-integration (Niko) -- reference only, unmodified
visual_grasp/   visual_grasp main branch -- unmodified
integration/    glue code written for this repo (mock OH bridge + orchestrator)
CREDITS.md      exact source commits + what was changed
```

## Known gaps / next steps

- **Real OpenHarmony-in-the-loop**: swap `integration/oh_bridge_mock.py` for
  the real `oh_bridge/` chain once it can run somewhere with OpenHarmony QEMU
  already working (Tim's machine, not rebuilt here).
- **Upstream the semantic-intent patch**: the `llm/backend/*.py` changes in
  this repo should go back to Mingrui as a PR against `robot-command-demo`
  rather than living only as a fork here.
- **`real_piper` backend**: `visual_grasp`'s `main` branch (`piper_backend.py`)
  and `zenghao720` branch (`real_piper_backend.py` + `ros1_gateway.py`)
  independently implement the real-hardware boundary and conflict with each
  other (`multitask/bridge.py`, `tests/test_bridge.py`). This demo sidesteps
  that entirely by only using `backend=sim_mujoco`, which is unaffected by
  the conflict -- but real-hardware work still needs that reconciled.
