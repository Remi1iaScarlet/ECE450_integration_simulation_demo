"""CLI entry point for natural-language multitask commands.

Example:
    python -m multitask.run_nl "把杯子放到碗里"
    python -m multitask.run_nl "clear table into bowl" --dry-run
"""
import argparse
import json
import os

os.environ.setdefault("MUJOCO_GL", "egl")

from .nl import DEFAULT_NL_CFG, NLParseError, execute_parsed_task, parse_nl


def _load_cfg_for_parse():
    try:
        from .config import load_config
        return load_config()
    except ImportError:
        return DEFAULT_NL_CFG


def main():
    parser = argparse.ArgumentParser(
        description="Parse a Chinese/English natural-language command and run a multitask task."
    )
    parser.add_argument("command", help="e.g. '把杯子放到碗里', 'pick cup', 'clear table into bowl'")
    parser.add_argument("--policy", default="nearest")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the parsed structured task without running MuJoCo")
    parser.add_argument("--record-cam", default="world_cam", choices=["world_cam", "wrist_cam"],
                        help="camera for the evidence gif/png")
    parser.add_argument("--perception", default=None, choices=["yolo", "sim_gt"],
                        help="override detector.backend; sim_gt uses MuJoCo body poses")
    parser.add_argument("--annotate", action="store_true",
                        help="draw YOLO detection boxes on the recorded frames")
    parser.add_argument("--no-evidence", action="store_true")
    args = parser.parse_args()

    cfg = _load_cfg_for_parse()
    try:
        parsed = parse_nl(args.command, cfg=cfg, policy=args.policy)
    except NLParseError as exc:
        parser.error(str(exc))

    print("=== parsed task (JSON) ===")
    print(json.dumps(parsed.to_dict(), ensure_ascii=False, indent=2))
    if args.dry_run:
        return 0

    from .config import load_config
    from .executor import TaskExecutor, _save_evidence
    from .world import SimWorld

    cfg = load_config()
    if args.perception is not None:
        cfg["detector"]["backend"] = args.perception
    labels = list(cfg["objects"]["graspable"]) + list(cfg["objects"]["containers"])
    world = SimWorld.load(record=not args.no_evidence, record_cam=args.record_cam,
                          annotate_labels=(labels if args.annotate else ()),
                          annotate_conf=cfg["detector"]["confidence_threshold"])
    summary = execute_parsed_task(parsed, TaskExecutor(world, cfg))

    print("\n=== task result (JSON) ===")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if not args.no_evidence:
        name = f"nl_{parsed.task}_{parsed.source_label or 'table'}"
        if args.record_cam != "world_cam":
            name += f"_{args.record_cam}"
        if args.annotate:
            name += "_yolo"
        _save_evidence(world, name)
    return 0 if summary["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
