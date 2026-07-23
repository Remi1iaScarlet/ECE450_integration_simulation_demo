"""Rule-based multi-task grasp layer over the MuJoCo visual-grasp pipeline.

Wraps the existing sim/ perception + IK/control code into an object registry,
reusable primitives, a task library, and a finite-state executor. See
TODO_multitask_yolo_grasp.md for the roadmap.
"""
from .nl import NLParseError, ParsedTask, execute_parsed_task, parse_nl

__all__ = [
    "SimWorld", "TaskExecutor", "load_config",
    "pick", "place_at", "place_into", "clear_table",
    "ParsedTask", "NLParseError", "parse_nl", "execute_parsed_task",
]


def __getattr__(name):
    if name == "load_config":
        from .config import load_config
        return load_config
    if name == "TaskExecutor":
        from .executor import TaskExecutor
        return TaskExecutor
    if name == "SimWorld":
        from .world import SimWorld
        return SimWorld
    if name in {"pick", "place_at", "place_into", "clear_table"}:
        from . import task_library
        return getattr(task_library, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
