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
    model: str | None = "real",
) -> dict:
    """Convert a robot-command-demo queue item's llm_output into a
    visual_grasp bridge JSON command.

    ``perception`` defaults to "sim_gt" (deterministic ground-truth object
    poses) rather than "yolo", matching visual_grasp's own guidance for
    reproducible demos (see visual_grasp/devlog/DEVLOG.md, 2026-07-08 entry
    on the YOLO clear_table +y bottle edge case).
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
