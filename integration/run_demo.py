#!/usr/bin/env python3
"""End-to-end demo: LLM entry point -> MuJoCo grasp.

    text command
        -> llm/backend (Mingrui Li's Intent mapper + validator)
        -> oh_bridge_mock.py (stand-in for the real OpenHarmony/ROS2/ROS1 hop
           in oh_bridge/, see that module's docstring)
        -> visual_grasp/multitask/bridge.py (chuhan + runhanw + team)
        -> MuJoCo simulation

Three ways to supply the command, from most to least "real":

  1. --from-queue     Poll a *running* robot-command-demo FastAPI instance
                       (llm/) for its currently-posted, human-approved queue
                       item. This is the real flow: you type a command in
                       the web UI at localhost:8000/app, approve it in
                       localhost:8000/queue, then run this script. Requires
                       `uvicorn backend.main:app` running in llm/ with a real
                       LLM_API_KEY configured (see llm/DEPLOY.md).

  2. --command TEXT    Call the LLM mapper directly, in-process, skipping the
                       FastAPI queue/approval step. Still requires a real
                       LLM_API_KEY (Anthropic or OpenAI) in the environment.

  3. --llm-json JSON   Skip the LLM entirely and supply an already-mapped
                       {"intent": ..., "parameters": {...}} object. This is
                       the only mode that needs no API key and no running
                       server -- useful for testing the bridge/MuJoCo leg in
                       isolation, e.g. on a machine with no network access.

Example (offline, no API key, matches DEMOS.md section 3b):

    python3 integration/run_demo.py --llm-json \\
        '{"intent": "place_into", "parameters": {"source_label": "cup", "container_label": "bowl"}}'
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "llm"))
sys.path.insert(0, str(REPO_ROOT / "visual_grasp"))

from oh_bridge_mock import UnroutableCommand, llm_item_to_bridge_command  # noqa: E402


def _validated_llm_output(intent: str, parameters: dict) -> dict:
    """Run a manually-supplied intent/parameters pair through the real
    backend.validator.py whitelist so --llm-json can't bypass validation."""
    from backend.schema import LLMOutput
    from backend.validator import validate_llm_output

    llm_output = LLMOutput(
        is_valid=True, intent=intent, parameters=parameters,
        confidence=1.0, reason="supplied via --llm-json",
    )
    validation = validate_llm_output(llm_output)
    if not validation.passed:
        print(f"validation failed: {validation.error}", file=sys.stderr)
        sys.exit(2)
    return {"intent": llm_output.intent.value, "parameters": llm_output.parameters}


def _from_command_text(text: str) -> dict:
    from backend.llm_mapper import map_command_to_intent, get_fallback_output
    from backend.validator import validate_llm_output

    llm_output = map_command_to_intent(text) or get_fallback_output("LLM call failed")
    validation = validate_llm_output(llm_output)
    if not validation.passed:
        print(f"validation failed: {validation.error}", file=sys.stderr)
        sys.exit(2)
    if llm_output.intent.value == "invalid":
        print(f"LLM marked this command invalid: {llm_output.reason}", file=sys.stderr)
        sys.exit(2)
    return {"intent": llm_output.intent.value, "parameters": llm_output.parameters}


def _from_queue(api_base: str) -> tuple[dict, int | None]:
    import urllib.request

    with urllib.request.urlopen(f"{api_base}/api/queue/current-posted") as resp:
        payload = json.load(resp)
    item = payload.get("item")
    if item is None:
        print("no posted queue item; submit + approve a command at "
              f"{api_base}/app and {api_base}/queue first", file=sys.stderr)
        sys.exit(2)
    llm_output = item["llm_output"]
    return {"intent": llm_output["intent"], "parameters": llm_output["parameters"]}, item["id"]


def _mark_queue_item(api_base: str, item_id: int, outcome: str) -> None:
    import urllib.request

    req = urllib.request.Request(f"{api_base}/api/queue/{item_id}/{outcome}", method="POST")
    urllib.request.urlopen(req)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--from-queue", action="store_true",
                         help="poll a running llm/ FastAPI instance for the posted queue item")
    source.add_argument("--command", help="natural-language command, sent to the real LLM mapper")
    source.add_argument("--llm-json", help='{"intent": ..., "parameters": {...}}, no LLM call')
    parser.add_argument("--api-base", default="http://127.0.0.1:8000",
                         help="llm/ FastAPI base URL, used with --from-queue")
    parser.add_argument("--backend", default="sim_mujoco", choices=["sim_mujoco"],
                         help="only sim_mujoco is wired up in this demo repo; real_piper needs "
                              "the real ROS1 gateway, see oh_bridge_mock.py docstring")
    parser.add_argument("--perception", default="sim_gt", choices=["sim_gt", "yolo"])
    parser.add_argument("--model", default="real", choices=["menagerie", "real"])
    parser.add_argument("--no-evidence", action="store_true", help="skip saving a GIF/PNG of the run")
    parser.add_argument("--mp4", action="store_true",
                         help="save a full-resolution MP4 instead of the default downscaled GIF "
                              "(for filming a demo); writes to visual_grasp/multitask/evidence/")
    parser.add_argument("--record-cam", default="world_cam", choices=["world_cam", "wrist_cam"],
                         help="which camera's frames to save as evidence")
    parser.add_argument("--dry-run", action="store_true", help="validate + build the bridge command, skip MuJoCo")
    args = parser.parse_args()

    queue_item_id = None
    if args.from_queue:
        llm_output, queue_item_id = _from_queue(args.api_base)
    elif args.command:
        llm_output = _from_command_text(args.command)
    else:
        try:
            parsed = json.loads(args.llm_json)
        except json.JSONDecodeError as exc:
            parser.error(f"--llm-json is not valid JSON: {exc}")
        llm_output = _validated_llm_output(parsed.get("intent"), parsed.get("parameters", {}))

    try:
        bridge_command = llm_item_to_bridge_command(
            llm_output,
            request_id=f"demo_{queue_item_id or 'local'}",
            backend=args.backend,
            perception=args.perception,
            model=args.model,
        )
    except UnroutableCommand as exc:
        print(str(exc), file=sys.stderr)
        if queue_item_id is not None:
            _mark_queue_item(args.api_base, queue_item_id, "failed")
        return 2

    from multitask import bridge  # visual_grasp/multitask/bridge.py

    evidence_kwargs = {}
    if args.mp4 and not args.dry_run and not args.no_evidence:
        from video_export import mp4_evidence_saver
        evidence_kwargs["evidence_saver"] = mp4_evidence_saver()

    result = bridge.run_bridge(
        bridge_command,
        dry_run=args.dry_run,
        no_evidence=args.no_evidence,
        record_cam=args.record_cam,
        **evidence_kwargs,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2, default=str))

    if queue_item_id is not None:
        _mark_queue_item(args.api_base, queue_item_id, "executed" if result.get("success") else "failed")

    return 0 if result.get("success", args.dry_run) else 1


if __name__ == "__main__":
    raise SystemExit(main())
