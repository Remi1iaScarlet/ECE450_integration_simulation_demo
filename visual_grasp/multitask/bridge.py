"""Bridge-ready JSON command entry point for the multitask executor.

This module is intentionally thin: it validates and normalizes commands from a
future UI / OpenHarmony / ROS bridge, then optionally dispatches them to the
existing TaskExecutor. Dry-run mode is dependency-light and does not import
MuJoCo, so interface tests can run on machines without the sim stack installed.
"""
from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
from typing import Any, Mapping


SCHEMA_VERSION = "visual_grasp.bridge.v1"
SUPPORTED_TASKS = {"pick", "place_at", "place_into", "clear_table"}
TASK_ALIASES = {
    "grab": "pick",
    "grasp": "pick",
    "put": "place_at",
    "place": "place_at",
    "put_into": "place_into",
    "clear": "clear_table",
}
BACKEND_ALIASES = {
    "sim": "sim_mujoco",
    "simulation": "sim_mujoco",
    "mujoco": "sim_mujoco",
    "real": "real_piper",
    "piper": "real_piper",
}


class BridgeCommandError(ValueError):
    """Raised when a bridge command is malformed or unsupported."""


def load_command(raw: str | Mapping[str, Any] | None = None, path: str | None = None) -> dict:
    """Load a command from a JSON string, dict-like object, or JSON file."""
    if path:
        return json.loads(Path(path).read_text())
    if raw is None:
        return {}
    if isinstance(raw, Mapping):
        return dict(raw)
    try:
        return json.loads(raw)
    except json.JSONDecodeError as exc:
        raise BridgeCommandError(f"command must be valid JSON: {exc.msg}") from exc


def normalize_command(payload: Mapping[str, Any]) -> dict:
    """Validate a raw bridge command and return the canonical task shape."""
    raw_task = _as_str(payload.get("task") or payload.get("action") or payload.get("intent"))
    task = TASK_ALIASES.get(raw_task, raw_task)
    if task not in SUPPORTED_TASKS:
        raise BridgeCommandError(
            f"unsupported task {raw_task!r}; supported tasks: {sorted(SUPPORTED_TASKS)}"
        )

    source = _as_optional_str(
        payload.get("source")
        or payload.get("source_label")
        or payload.get("object")
        or (payload.get("target") if task == "pick" else None)
    )
    container = _as_optional_str(
        payload.get("container")
        or payload.get("container_label")
        or (payload.get("target") if task in {"place_into", "clear_table"} else None)
    )
    labels = _labels(payload.get("labels") or payload.get("objects"))
    xyz = _xyz(payload.get("xyz") or payload.get("target_xyz") or payload.get("place_xyz"))

    backend = _as_optional_str(payload.get("backend") or "sim_mujoco")
    backend = BACKEND_ALIASES.get(backend, backend)
    perception = _as_optional_str(payload.get("perception") or payload.get("perception_backend") or "yolo")
    policy = _as_optional_str(payload.get("policy") or "nearest")
    speed = payload.get("speed", None)
    if speed is not None:
        speed = _float(speed, "speed")
    armed = _strict_bool(payload.get("armed", False), "armed")
    model = _as_optional_str(payload.get("model"))
    provider_factory = _as_optional_str(payload.get("provider_factory"))
    planner_factory = _as_optional_str(payload.get("planner_factory"))

    _validate_task_args(task, source, container, xyz, labels)
    if backend == "real_piper" and perception == "sim_gt":
        raise BridgeCommandError(
            "real_piper cannot use sim_gt perception; inject a real perception provider")

    return {
        "schema_version": SCHEMA_VERSION,
        "task": task,
        "source_label": source,
        "container_label": container,
        "labels": labels,
        "target_xyz": xyz,
        "backend": backend,
        "perception": perception,
        "policy": policy,
        "speed": speed,
        "armed": armed,
        "model": model,
        "provider_factory": provider_factory,
        "planner_factory": planner_factory,
    }


def dry_run_result(command: Mapping[str, Any]) -> dict:
    """Return a bridge-shaped success result without running robot/sim code."""
    return {
        "schema_version": SCHEMA_VERSION,
        "success": True,
        "dry_run": True,
        "message": "command validated; execution skipped",
        "command": dict(command),
        "dispatch": _dispatch_preview(command),
    }


def execute_command(
        command: Mapping[str, Any], no_evidence: bool = False,
        record_cam: str = "world_cam", annotate: bool = False,
        *, allow_hardware: bool = False, backend_factory=None,
        executor_factory=None, perception_factory=None, planner_factory=None,
        config_loader=None, sim_world_factory=None,
        sim_executor_factory=None, evidence_saver=None) -> dict:
    """Dispatch a normalized command to sim or an explicitly armed real seam."""
    backend = command["backend"]
    if backend == "real_piper":
        return _execute_real_piper(
            command, allow_hardware=allow_hardware,
            backend_factory=backend_factory,
            executor_factory=executor_factory,
            perception_factory=perception_factory,
            planner_factory=planner_factory)
    if backend != "sim_mujoco":
        raise BridgeCommandError(
            f"unsupported backend {backend!r}; use sim_mujoco or real_piper"
        )

    return _execute_sim_mujoco(
        command, no_evidence=no_evidence, record_cam=record_cam,
        annotate=annotate, config_loader=config_loader,
        world_factory=sim_world_factory,
        executor_factory=sim_executor_factory,
        evidence_saver=evidence_saver)


def _execute_real_piper(
        command: Mapping[str, Any], *, allow_hardware: bool,
        backend_factory, executor_factory, perception_factory,
        planner_factory) -> dict:
    """Run the real task contract through gated, explicit dependency seams."""
    if command.get("perception") == "sim_gt":
        raise BridgeCommandError("real_piper cannot use sim_gt perception")
    if command.get("armed") is not True:
        raise BridgeCommandError(
            "real_piper execution requires command field armed=true")
    if allow_hardware is not True:
        raise BridgeCommandError(
            "real_piper execution requires API/CLI allow_hardware=true")
    if executor_factory is None and command.get("task") != "pick":
        raise BridgeCommandError(
            "the repository default RealPiperTaskExecutor currently supports "
            "only pick; inject a commissioned executor_factory for other tasks")
    from .piper_backend import issue_hardware_authorization
    authorization = issue_hardware_authorization(
        armed=command.get("armed"), allow_hardware=allow_hardware)
    if backend_factory is None:
        from .piper_backend import create_default_piper_backend
        backend_factory = create_default_piper_backend

    resources = []
    primary_error = None
    try:
        # The authorization capability exists before any backend construction.
        backend = backend_factory(command, authorization=authorization)
        resources.append(backend)

        if perception_factory is None:
            perception_factory = _configured_factory(
                command.get("provider_factory"), "provider")
        perception = perception_factory(command)
        resources.append(perception)

        if executor_factory is None:
            if planner_factory is None:
                planner_factory = _configured_factory(
                    command.get("planner_factory"), "planner")
            planner = planner_factory(command)
            resources.append(planner)
            from .real_piper_executor import create_default_real_piper_executor
            executor = create_default_real_piper_executor(
                backend, perception, planner, command)
        else:
            executor = executor_factory(backend, perception, command)
        resources.append(executor)

        _preflight_executor(executor, command)
        backend.connect()
        backend.enable()
        return _dispatch_executor(executor, command)
    except Exception as exc:
        primary_error = exc
        raise
    finally:
        cleanup_errors = []
        seen = set()
        for resource in reversed(resources):
            if id(resource) in seen:
                continue
            seen.add(id(resource))
            close = getattr(resource, "close", None)
            if not callable(close):
                continue
            try:
                close()
            except Exception as cleanup_error:
                cleanup_errors.append(cleanup_error)
        if cleanup_errors:
            if primary_error is None:
                raise cleanup_errors[0]
            if hasattr(primary_error, "add_note"):
                for cleanup_error in cleanup_errors:
                    primary_error.add_note(
                        f"real_piper resource close failed: {cleanup_error}")


def _execute_sim_mujoco(
        command: Mapping[str, Any], *, no_evidence: bool,
        record_cam: str, annotate: bool, config_loader=None,
        world_factory=None, executor_factory=None, evidence_saver=None) -> dict:
    """Construct sim runtime with the executor's model/scene/IK selection rules."""
    if config_loader is None:
        from .config import load_config
        config_loader = load_config
    if world_factory is None:
        from .world import SimWorld
        world_factory = SimWorld.load
    if executor_factory is None:
        from .executor import TaskExecutor
        executor_factory = TaskExecutor
    if evidence_saver is None and not no_evidence:
        from .executor import _save_evidence
        evidence_saver = _save_evidence

    cfg = config_loader()
    if command["perception"]:
        cfg["detector"]["backend"] = command["perception"]
    scene_path, analytic_ik = _apply_executor_model_config(
        cfg, command.get("model"))
    labels = list(cfg["objects"]["graspable"]) + list(
        cfg["objects"]["containers"])
    world = world_factory(
        scene=scene_path,
        record=not no_evidence,
        record_cam=record_cam,
        annotate_labels=(labels if annotate else ()),
        annotate_conf=cfg["detector"]["confidence_threshold"],
        use_analytic_ik=analytic_ik,
    )
    executor = executor_factory(world, cfg)
    summary = _dispatch_executor(executor, command)
    if not no_evidence:
        evidence_saver(
            world,
            f"bridge_{command['task']}_{command['source_label'] or 'table'}")
    return summary


def _apply_executor_model_config(cfg: dict, model_override=None):
    """Mirror TaskExecutor CLI model.active, scene, IK, and override parsing."""
    model_cfg = cfg.get("model", {})
    active = model_override or model_cfg.get("active", "menagerie")
    canonical_active = "piper_real" if active == "real" else active
    config_active = "real" if canonical_active == "piper_real" else canonical_active
    selected = model_cfg.get(config_active, {})
    if not isinstance(selected, Mapping):
        raise BridgeCommandError(
            f"model configuration {config_active!r} must be an object")
    scene_path = None
    if selected.get("scene"):
        scene_path = Path(__file__).resolve().parent.parent / selected["scene"]
    analytic_ik = bool(selected.get("analytic_ik", False))
    if selected.get("scan_poses"):
        cfg["detector"]["scan_poses"] = selected["scan_poses"]
    for section in ("grasp", "place", "scene", "detector"):
        if isinstance(selected.get(section), dict):
            cfg[section].update(selected[section])
    return scene_path, analytic_ik


def _configured_factory(path: str | None, role: str):
    if not path:
        raise BridgeCommandError(
            f"real_piper {role} factory is not configured; provide "
            f"{role}_factory injection or an import path")
    module_name, separator, attribute_path = path.partition(":")
    if not separator or not module_name or not attribute_path:
        raise BridgeCommandError(
            f"{role} factory path must use 'module:callable' format")
    try:
        value = importlib.import_module(module_name)
        for part in attribute_path.split("."):
            value = getattr(value, part)
    except (ImportError, AttributeError) as exc:
        raise BridgeCommandError(
            f"cannot import {role} factory {path!r}: {exc}") from exc
    if not callable(value):
        raise BridgeCommandError(f"configured {role} factory {path!r} is not callable")
    return value


def _dispatch_executor(executor, command: Mapping[str, Any]) -> dict:
    """Reuse the existing TaskExecutor public task-call contract."""
    task = command["task"]
    if task == "pick":
        return executor.pick(command["source_label"], command["policy"])
    if task == "place_at":
        return executor.place_at(
            command["source_label"], command["target_xyz"], command["policy"])
    if task == "place_into":
        return executor.place_into(
            command["source_label"], command["container_label"],
            command["target_xyz"], command["policy"])
    return executor.clear_table(
        command["labels"], command["container_label"],
        command["target_xyz"], command["policy"])


def _preflight_executor(executor, command: Mapping[str, Any]) -> None:
    """Reject missing executor/runtime contracts before hardware connection."""
    task_method = getattr(executor, command["task"], None)
    if not callable(task_method):
        raise BridgeCommandError(
            f"executor does not implement callable task {command['task']!r}")
    preflight = getattr(executor, "preflight", None)
    if callable(preflight):
        preflight()


def run_bridge(raw: str | Mapping[str, Any] | None = None, path: str | None = None,
               dry_run: bool = False, **execute_kwargs) -> dict:
    """Load, normalize, and either dry-run or execute a bridge command."""
    command = normalize_command(load_command(raw, path))
    if dry_run:
        return dry_run_result(command)
    return execute_command(command, **execute_kwargs)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate or run a visual_grasp bridge JSON command.")
    parser.add_argument("command", nargs="?", help="JSON command string")
    parser.add_argument("--file", help="read command JSON from a file")
    parser.add_argument("--task", choices=sorted(SUPPORTED_TASKS), help="build a command without JSON")
    parser.add_argument("--source", help="source object label, e.g. cup")
    parser.add_argument("--target", help="container label for place_into, e.g. bowl")
    parser.add_argument("--xyz", help="target xyz for place_at/place_into, e.g. 0.4,0.1,0.17")
    parser.add_argument("--labels", help="clear_table labels, comma-separated")
    parser.add_argument("--backend", default=None, help="sim_mujoco, sim, real_piper")
    parser.add_argument("--perception", default=None, choices=["yolo", "sim_gt"])
    parser.add_argument("--model", default=None,
                        choices=["menagerie", "piper_real", "real"])
    parser.add_argument(
        "--provider-factory", default=None,
        help="real provider factory import path, e.g. pkg.module:create_provider")
    parser.add_argument(
        "--planner-factory", default=None,
        help="real planner factory import path, e.g. pkg.module:create_planner")
    parser.add_argument("--policy", default=None)
    parser.add_argument("--speed", type=float, default=None)
    parser.add_argument("--dry-run", action="store_true", help="validate only; do not import/run MuJoCo")
    parser.add_argument(
        "--armed", action="store_true",
        help="first real_piper opt-in; still requires --allow-hardware")
    parser.add_argument(
        "--allow-hardware", action="store_true",
        help="second real_piper opt-in; ignored by sim and dry-run")
    parser.add_argument("--no-evidence", action="store_true")
    parser.add_argument("--record-cam", default="world_cam", choices=["world_cam", "wrist_cam"])
    parser.add_argument("--annotate", action="store_true")
    args = parser.parse_args(argv)

    payload = load_command(args.command, args.file) if (args.command or args.file) else {}
    if args.task:
        payload["task"] = args.task
    if args.source:
        payload["source"] = args.source
    if args.target:
        payload["target"] = args.target
    if args.xyz:
        payload["xyz"] = _xyz(args.xyz)
    if args.labels:
        payload["labels"] = _labels(args.labels)
    if args.backend:
        payload["backend"] = args.backend
    if args.perception:
        payload["perception"] = args.perception
    if args.model:
        payload["model"] = args.model
    if args.provider_factory:
        payload["provider_factory"] = args.provider_factory
    if args.planner_factory:
        payload["planner_factory"] = args.planner_factory
    if args.policy:
        payload["policy"] = args.policy
    if args.speed is not None:
        payload["speed"] = args.speed
    if args.armed:
        payload["armed"] = True

    try:
        result = run_bridge(
            payload,
            dry_run=args.dry_run,
            no_evidence=args.no_evidence,
            record_cam=args.record_cam,
            annotate=args.annotate,
            allow_hardware=args.allow_hardware,
        )
    except BridgeCommandError as exc:
        print(json.dumps({"success": False, "error": str(exc)}, ensure_ascii=False, indent=2))
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("success", False) else 1


def _dispatch_preview(command: Mapping[str, Any]) -> dict:
    task = command["task"]
    executor_name = (
        "RealPiperTaskExecutor"
        if command.get("backend") == "real_piper"
        else "TaskExecutor"
    )
    if task == "pick":
        return {"call": f"{executor_name}.pick", "args": [command["source_label"], command["policy"]]}
    if task == "place_at":
        return {
            "call": f"{executor_name}.place_at",
            "args": [command["source_label"], command["target_xyz"], command["policy"]],
        }
    if task == "place_into":
        return {
            "call": f"{executor_name}.place_into",
            "args": [
                command["source_label"],
                command["container_label"],
                command["target_xyz"],
                command["policy"],
            ],
        }
    return {
        "call": f"{executor_name}.clear_table",
        "args": [command["labels"], command["container_label"], command["target_xyz"], command["policy"]],
    }


def _validate_task_args(task: str, source: str | None, container: str | None,
                        xyz: list[float] | None, labels: list[str]) -> None:
    if task in {"pick", "place_at", "place_into"} and not source:
        raise BridgeCommandError(f"{task} requires a source object label")
    if task == "place_at" and xyz is None:
        raise BridgeCommandError("place_at requires target_xyz / xyz")
    if task == "place_into" and not container and xyz is None:
        raise BridgeCommandError("place_into requires container label or target_xyz / xyz")
    if task == "clear_table" and not labels:
        raise BridgeCommandError("clear_table requires labels / objects")


def _labels(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [v.strip() for v in value.split(",") if v.strip()]
    if isinstance(value, (list, tuple)):
        labels = [_as_str(v) for v in value]
        if not all(labels):
            raise BridgeCommandError("labels must be non-empty strings")
        return labels
    raise BridgeCommandError("labels must be a comma-separated string or a list")


def _xyz(value: Any) -> list[float] | None:
    if value is None:
        return None
    if isinstance(value, str):
        parts = [v.strip() for v in value.split(",")]
    elif isinstance(value, (list, tuple)):
        parts = list(value)
    else:
        raise BridgeCommandError("xyz must be 'x,y,z' or a length-3 list")
    if len(parts) != 3:
        raise BridgeCommandError("xyz must have exactly 3 values")
    return [_float(v, "xyz") for v in parts]


def _as_str(value: Any) -> str:
    if value is None:
        return ""
    if not isinstance(value, str):
        raise BridgeCommandError(f"expected string, got {type(value).__name__}")
    return value.strip()


def _as_optional_str(value: Any) -> str | None:
    text = _as_str(value)
    return text or None


def _float(value: Any, name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise BridgeCommandError(f"{name} must be numeric") from exc


def _strict_bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise BridgeCommandError(f"{name} must be a JSON boolean")
    return value


if __name__ == "__main__":
    raise SystemExit(main())
