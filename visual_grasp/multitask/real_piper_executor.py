"""Dependency-injected real Piper task executor.

This module owns no camera, SDK, CAN, or planner construction.  Its concrete
pick path composes configured perception and planning providers with the
feedback-guarded hardware backend.
"""
from __future__ import annotations

from dataclasses import asdict, is_dataclass

from .motion_plan import (MotionPlan, MotionPlanContractError,
                          validate_motion_plan_contract_type,
                          validate_motion_plan_instance)


class RealPiperExecutorError(RuntimeError):
    """Raised when a required real perception/planning contract is unavailable."""


class RealPiperTaskExecutor:
    """Compose locate -> MotionPlan build -> explicit grasp execution."""

    def __init__(self, backend, perception, planner, command=None):
        self.backend = backend
        self.perception = perception
        self.planner = planner
        self.command = dict(command or {})

    def preflight(self) -> None:
        """Validate factories, callable seams, and the neutral plan contract."""
        self._required_pick_methods()
        validate_motion_plan_contract_type()
        declared_plan_type = getattr(self.planner, "motion_plan_type", None)
        if declared_plan_type is not None and declared_plan_type is not MotionPlan:
            raise RealPiperExecutorError(
                "real planner motion_plan_type must be "
                "multitask.motion_plan.MotionPlan")
        for role, resource in (("perception", self.perception),
                               ("planner", self.planner)):
            resource_preflight = getattr(resource, "preflight", None)
            if callable(resource_preflight):
                try:
                    resource_preflight()
                except Exception as exc:
                    if hasattr(exc, "add_note"):
                        exc.add_note(f"real {role} preflight failed")
                    raise

    def _required_pick_methods(self):
        locate = getattr(self.perception, "locate", None)
        if not callable(locate):
            raise RealPiperExecutorError(
                "real perception provider must implement locate(source_label)")
        build_plan = getattr(self.planner, "build_pick_motion_plan", None)
        if not callable(build_plan):
            raise RealPiperExecutorError(
                "real planner must implement build_pick_motion_plan(...)")
        return locate, build_plan

    def pick(self, source_label, policy="nearest"):
        locate, build_plan = self._required_pick_methods()

        observation = locate(source_label)
        if observation is None:
            raise RealPiperExecutorError(
                f"real perception did not locate source object {source_label!r}")
        motion_plan = build_plan(
            observation, source_label=source_label, policy=policy)
        try:
            motion_plan = validate_motion_plan_instance(motion_plan)
        except MotionPlanContractError as exc:
            raise RealPiperExecutorError(
                f"invalid real planner MotionPlan: {exc}") from exc

        report = self.backend.execute_grasp_plan(motion_plan)
        report_data = asdict(report) if is_dataclass(report) else report
        return {
            "success": True,
            "message": "real Piper pick MotionPlan executed",
            "data": {
                "source_label": source_label,
                "policy": policy,
                "motion_report": report_data,
            },
        }

    def place_at(self, *args, **kwargs):
        raise RealPiperExecutorError(
            "real_piper place_at is not configured or commissioned")

    def place_into(self, *args, **kwargs):
        raise RealPiperExecutorError(
            "real_piper place_into is not configured or commissioned")

    def clear_table(self, *args, **kwargs):
        raise RealPiperExecutorError(
            "real_piper clear_table is not configured or commissioned")


def create_default_real_piper_executor(
        backend, perception, planner, command=None) -> RealPiperTaskExecutor:
    return RealPiperTaskExecutor(backend, perception, planner, command)
