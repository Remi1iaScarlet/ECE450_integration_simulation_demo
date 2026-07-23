# Credits

This repo integrates three ECE450 capstone teammates' repositories into one
runnable LLM-to-MuJoCo simulation demo. No submodules are used (by choice,
see the main README) -- each source tree was copied verbatim at the commit
below, then only the files listed under "Modifications" were changed.

## `llm/` -- robot-command-demo

- Author: Mingrui Li ([MingruiLiBigCat](https://github.com/MingruiLiBigCat))
- Source: https://github.com/MingruiLiBigCat/robot-command-demo
- Commit: `d4ed7caae9c4552a95ecf85bc07c4d456a5ae4e7` (2026-07-12, "update")
- Role: natural-language -> structured-intent LLM mapper, FastAPI queue +
  human-approval web UI.

**Modifications** (2026-07-23, for this integration repo):
- `backend/schema.py` -- added semantic `Intent` members (`pick`, `place_at`,
  `place_into`, `clear_table`) alongside the original 5 primitives.
- `backend/validator.py` -- added parameter validation and
  `/visual_grasp/task` routing for the new semantic intents; primitive
  intents still route to `/stage2_arm/move` exactly as before.
- `backend/llm_mapper.py` -- extended `SYSTEM_PROMPT` with the semantic
  intents and CN/EN examples (e.g. "put the cup in the bowl" / "把杯子放到碗里").
- Everything else in `llm/` (FastAPI app, queue store, static UI, deploy
  docs) is unmodified.

## `oh_bridge/` -- capstone-openharmony-integration

- Author: NikKk0o / Tim Yu ([NikKk0o](https://github.com/NikKk0o))
- Source: https://github.com/NikKk0o/capstone-openharmony-integration
- Commit: `379d6c360268d29ee00608e13aed4bd8061059fa` (2026-07-23, "Add complete
  launch and connection guide")
- Role: **reference only** in this repo. This is the real OpenHarmony(QEMU)
  -> ROS2 Humble -> ROS1 Noetic bridge architecture; it is not executed here.
  `integration/oh_bridge_mock.py` is a Python stand-in for this chain, kept
  to the same JSON shape (`visual_grasp.bridge.v1`) so it can be swapped for
  the real chain later. See `oh_bridge/docs/FULL_LAUNCH_AND_CONNECTION_GUIDE.md`
  for the real, hardware-attached setup this stands in for, and this repo's
  README ("Scope and honesty") for why it isn't reproduced here.
- Unmodified copy.

## `visual_grasp/` -- visual_grasp

- Authors: chuhan (Remi1iaScarlet), runhanw, Valttery, and team
- Source: https://github.com/runhanw/visual_grasp (branch `main`)
- Commit: `e79999ab11420622c7c1248a5a5e5969752e0c4c` (2026-07-22, "Add
  validated grasp planning and Piper backend")
- Role: the MuJoCo simulation, semantic task library (`multitask/`), and
  Piper IK/grasp planning this whole demo runs on top of.
- Unmodified copy. Note: the `visual_grasp` repo also has a `zenghao720`
  branch (ROS1 Piper gateway + Docker, by skywalkertzh) that conflicts with
  `main`'s real-hardware backend and is **not** included here -- this demo
  only uses the `sim_mujoco` backend, which is unaffected by that conflict.

## Integration glue (new in this repo)

- `integration/oh_bridge_mock.py`, `integration/run_demo.py` -- written for
  this integration repo; not copied from any of the three source repos.
