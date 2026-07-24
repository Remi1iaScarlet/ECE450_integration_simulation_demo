"""Stand-in for the OpenHarmony <-> ROS2 <-> ROS1 hop.

The real pipeline (see oh_bridge/docs/FULL_LAUNCH_AND_CONNECTION_GUIDE.md) is:

    LLM queue item
        -> OpenHarmony queue agent (running inside QEMU, agent/oh_queue_agent.py)
        -> ROS2 /stage2_arm/task_command
        -> stage2_adapter.py -> ROS2 /visual_grasp/command_json
        -> TCP bridge (port 5005)
        -> ROS1 /visual_grasp/command_json
        -> visual_grasp ROS1 gateway (multitask/ros1_gateway.py, real hardware only)

None of that (OpenHarmony QEMU, ROS2 Humble, ROS1 Noetic/Docker) is installed
on this machine -- see README.md "Scope and honesty" section for why. This
module replaces the whole chain with one in-process Python function so the
LLM-to-MuJoCo path can actually run and be demoed here. It deliberately keeps
the same JSON *shape* (visual_grasp.bridge.v1) that the real chain produces,
so swapping this module for the real ROS2/ROS1 hop later is a drop-in change,
not a redesign.
"""
from __future__ import annotations

from typing import Any, Mapping

SCHEMA_VERSION = "visual_grasp.bridge.v1"

# robot-command-demo Intent -> visual_grasp bridge task name.
# Keep in sync with llm/backend/validator.py:SEMANTIC_SERVICE_MAP.
INTENT_TO_TASK = {
    "pick": "pick",
    "place_at": "place_at",
    "place_into": "place_into",
    "clear_table": "clear_table",
}


class UnroutableCommand(ValueError):
    """Raised when a queue item's intent has no semantic-task mapping."""


def llm_item_to_bridge_command(
    llm_output: Mapping[str, Any],
    *,
    request_id: str,
    backend: str = "sim_mujoco",
    perception: str = "sim_gt",
    model: str | None = "menagerie",
) -> dict:
    """Convert a robot-command-demo queue item's llm_output into a
    visual_grasp bridge JSON command.

    ``perception`` defaults to "sim_gt" (deterministic ground-truth object
    poses) rather than "yolo", matching visual_grasp's own guidance for
    reproducible demos (see visual_grasp/devlog/DEVLOG.md, 2026-07-08 entry
    on the YOLO clear_table +y bottle edge case).

    ``model`` defaults to "menagerie", not visual_grasp's own config.yaml
    default of "real" (piper_real + analytic IK). Diagnosed here on
    2026-07-24: on piper_real, the pre-grasp approach in
    multitask/primitives.py:execute_grasp is a single-jump move_to (no
    interpolated waypoints, unlike carry_to), which can sweep the open
    gripper into the object before the close/lift sequence even starts.
    Once that happens the finger joints (joint7/joint8) get pinned past
    their own declared range and never separate again -- place_into looks
    like it succeeds (VERIFY_DONE only checks XY, not whether the object
    actually left the gripper) but the object stays stuck to the fingers.
    Interpolating just the pre-grasp approach removes the stuck-contact
    symptom, but then changes which analytic-IK branch gets selected for
    the following descend-to-grasp step (branch choice is seeded from
    "closest to current joint state"), which in turn failed to actually
    capture the object in testing. That is a deeper, unresolved issue in
    piper_real's analytic IK path -- not something to patch around here.
    menagerie (MuJoCo numeric IK) has neither problem: verified by direct
    contact inspection (not just the XY-tolerance check) that the object
    fully separates from the gripper and rests in the container. Pass
    model="real" explicitly if you specifically need the piper_real path.
    """
    intent = llm_output.get("intent")
    task = INTENT_TO_TASK.get(intent)
    if task is None:
        raise UnroutableCommand(
            f"intent {intent!r} has no semantic-task mapping; only "
            f"{sorted(INTENT_TO_TASK)} are routed to /visual_grasp/task"
        )

    params = llm_output.get("parameters") or {}

    print(f"[oh_bridge_mock] (stand-in for OH queue agent) forwarding request_id={request_id}")
    print(f"[oh_bridge_mock] (stand-in for stage2_adapter) task={task} params={params}")

    return {
        "schema_version": SCHEMA_VERSION,
        "request_id": request_id,
        "task": task,
        "source_label": params.get("source_label"),
        "container_label": params.get("container_label"),
        "labels": params.get("labels"),
        "target_xyz": params.get("target_xyz"),
        "backend": backend,
        "perception": perception,
        "model": model,
    }
