"""Task library: high-level tasks composed over the FSM executor.

Thin wrappers so callers (CLI, tests, a future bridge/LLM) can invoke a task by
name and get a structured result. Each creates/reuses a SimWorld and runs the
executor.

Note on containers: real container *detection* in sim is currently blocked by the
YOLO domain gap (a bowl reads as 'toilet' and suppresses the cup; see DEVLOG
2026-06-29). place_into/clear_table therefore accept an explicit `container_xyz`
to place at a known container location; the vision path (resolve container from
the registry) is wired and will work once a detectable container model exists.
"""
from .config import load_config
from .executor import TaskExecutor
from .world import SimWorld


def _exec(world, cfg):
    return TaskExecutor(world or SimWorld.load(record=True), cfg or load_config())


def pick(label, world=None, policy="nearest", cfg=None):
    """Detect, grasp, and lift `label`."""
    return _exec(world, cfg).pick(label, policy)


def place_at(label, xyz, world=None, policy="nearest", cfg=None):
    """Pick `label` and release it at base-frame coordinate `xyz`."""
    return _exec(world, cfg).place_at(label, xyz, policy)


def place_into(source_label, container_label=None, container_xyz=None,
               world=None, policy="nearest", cfg=None):
    """Pick `source_label` and release it into a container.

    Resolve the container by vision (`container_label`, or any detected container)
    or by a known `container_xyz`.
    """
    return _exec(world, cfg).place_into(source_label, container_label, container_xyz, policy)


def clear_table(labels, container_label=None, container_xyz=None,
                world=None, policy="nearest", cfg=None):
    """place_into each source label in turn; logs every attempt."""
    return _exec(world, cfg).clear_table(labels, container_label, container_xyz, policy)
