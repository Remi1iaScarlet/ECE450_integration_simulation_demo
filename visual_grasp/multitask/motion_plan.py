"""Dependency-light motion-plan contracts shared by sim and real execution.

This module intentionally imports no MuJoCo, detector, ROS, camera, or robot
SDK package.  Real execution can therefore validate the plan contract before
any hardware connection while the simulation planner remains free to re-export
the exact same classes for compatibility.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, fields
from enum import Enum
from typing import Any


class MotionPhase(str, Enum):
    TRANSIT = "TRANSIT"
    PRE_APPROACH = "PRE_APPROACH"
    FINAL_APPROACH = "FINAL_APPROACH"
    CLOSING = "CLOSING"
    LIFT = "LIFT"


@dataclass
class CollisionTrace:
    phase: str
    segment_name: str
    sample_index: int
    alpha: float
    q: Any
    geom_names: tuple
    body_names: tuple
    contact_category: str
    distance_m: float
    allowed: bool
    reason: str

    def public(self):
        return {
            "phase": self.phase, "segment_name": self.segment_name,
            "sample_index": self.sample_index, "alpha": round(self.alpha, 4),
            "q": [round(float(value), 5) for value in self.q],
            "geom_names": list(self.geom_names),
            "body_names": list(self.body_names),
            "contact_category": self.contact_category,
            "distance_m": round(self.distance_m, 6), "allowed": self.allowed,
            "reason": self.reason,
        }


@dataclass
class JointWaypoint:
    name: str
    q: Any


@dataclass
class MotionSegment:
    name: str
    phase: str
    waypoints: list
    collision_trace: list = field(default_factory=list)


@dataclass
class MotionPlan:
    start_q: Any
    segments: list
    direct_collision_trace: list = field(default_factory=list)
    used_detour: bool = False
    collision_scope: str = "static_and_self"

    def public(self):
        return {
            "start_q": [round(float(value), 5) for value in self.start_q],
            "used_detour": self.used_detour,
            "collision_scope": self.collision_scope,
            "segments": [
                {
                    "name": segment.name,
                    "phase": segment.phase,
                    "waypoints": [
                        {
                            "name": waypoint.name,
                            "q": [round(float(value), 5) for value in waypoint.q],
                        }
                        for waypoint in segment.waypoints
                    ],
                    "collision_trace": [
                        trace.public() for trace in segment.collision_trace
                    ],
                }
                for segment in self.segments
            ],
            "direct_collision_trace": [
                trace.public() for trace in self.direct_collision_trace
            ],
        }


class MotionPlanContractError(ValueError):
    """A planner returned an object outside the neutral execution contract."""


def validate_motion_plan_contract_type() -> None:
    """Fail if the imported neutral dataclass schema is not the commissioned one."""
    expected = {
        JointWaypoint: ("name", "q"),
        MotionSegment: ("name", "phase", "waypoints", "collision_trace"),
        MotionPlan: (
            "start_q", "segments", "direct_collision_trace", "used_detour",
            "collision_scope"),
    }
    for contract_type, expected_fields in expected.items():
        actual_fields = tuple(item.name for item in fields(contract_type))
        if actual_fields != expected_fields:
            raise MotionPlanContractError(
                f"{contract_type.__name__} contract fields are {actual_fields!r}; "
                f"expected {expected_fields!r}")


def validate_motion_plan_instance(plan: Any) -> MotionPlan:
    """Validate a generated plan's concrete type and dependency-light shape."""
    validate_motion_plan_contract_type()
    if not isinstance(plan, MotionPlan):
        raise MotionPlanContractError(
            "planner must return multitask.motion_plan.MotionPlan")
    _validate_joint_shape(plan.start_q, "motion_plan.start_q")
    if not isinstance(plan.segments, list) or not plan.segments:
        raise MotionPlanContractError(
            "motion_plan.segments must be a non-empty list")
    for segment_index, segment in enumerate(plan.segments):
        if not isinstance(segment, MotionSegment):
            raise MotionPlanContractError(
                f"motion_plan.segments[{segment_index}] must be MotionSegment")
        if not isinstance(segment.name, str) or not segment.name:
            raise MotionPlanContractError(
                f"motion_plan.segments[{segment_index}].name must be non-empty")
        if not isinstance(segment.phase, str) or not segment.phase:
            raise MotionPlanContractError(
                f"motion_plan.segments[{segment_index}].phase must be non-empty")
        if not isinstance(segment.waypoints, list) or len(segment.waypoints) < 2:
            raise MotionPlanContractError(
                f"motion_plan.segments[{segment_index}].waypoints must contain "
                "at least two JointWaypoint values")
        for waypoint_index, waypoint in enumerate(segment.waypoints):
            if not isinstance(waypoint, JointWaypoint):
                raise MotionPlanContractError(
                    f"motion_plan.segments[{segment_index}].waypoints"
                    f"[{waypoint_index}] must be JointWaypoint")
            if not isinstance(waypoint.name, str) or not waypoint.name:
                raise MotionPlanContractError(
                    f"waypoint {segment_index}:{waypoint_index} name must be "
                    "non-empty")
            _validate_joint_shape(
                waypoint.q, f"waypoint {segment_index}:{waypoint_index}.q")
    return plan


def _validate_joint_shape(values: Any, source: str) -> None:
    try:
        joints = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise MotionPlanContractError(
            f"{source} must contain six numeric joints") from exc
    if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
        raise MotionPlanContractError(
            f"{source} must contain six finite joints")
