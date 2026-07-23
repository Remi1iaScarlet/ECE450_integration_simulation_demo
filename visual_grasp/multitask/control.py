"""Dependency-light motion execution contracts shared by sim and hardware.

This module deliberately imports neither MuJoCo nor any robot SDK.  Hardware
code can therefore reuse the simulation trajectory configuration without
making import or construction a hardware action.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


class MotionSafetyError(RuntimeError):
    """Structured, loggable error raised by a fail-closed motion guard.

    The one-argument form is retained for the existing simulation guards.
    """

    def __init__(self, code: str, message: str | None = None,
                 details: Mapping[str, Any] | None = None):
        if message is None:
            message = str(code)
            code = "MOTION_SAFETY_GUARD"
        super().__init__(message)
        self.code = str(code)
        self.message = str(message)
        self.details = dict(details or {})

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": type(self).__name__,
            "code": self.code,
            "message": self.message,
            "details": dict(self.details),
        }


@dataclass(frozen=True)
class TrajectoryExecutionConfig:
    """Shared continuous-waypoint execution limits.

    The ``*_steps`` fields are used by MuJoCo.  The seconds/timeouts fields are
    used by the real backend.  Keeping one immutable contract avoids a second
    planner-specific configuration format.
    """

    max_joint_velocity_rad_s: float = 0.8
    max_following_error_rad: float = 0.12
    final_settle_steps: int = 100
    control_period_steps: int = 1
    gripper_settle_steps: int = 300
    min_table_clearance_m: float = 0.010
    control_period_s: float = 0.02
    feedback_timeout_s: float = 0.25
    waypoint_timeout_s: float = 15.0
    state_max_age_s: float = 0.25
    final_position_tolerance_rad: float = 0.02
    plan_start_tolerance_rad: float = 0.03

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any] | None):
        values = values or {}
        defaults = cls()
        return cls(**{
            field: cast(values.get(field, getattr(defaults, field)))
            for field, cast in (
                ("max_joint_velocity_rad_s", float),
                ("max_following_error_rad", float),
                ("final_settle_steps", int),
                ("control_period_steps", int),
                ("gripper_settle_steps", int),
                ("min_table_clearance_m", float),
                ("control_period_s", float),
                ("feedback_timeout_s", float),
                ("waypoint_timeout_s", float),
                ("state_max_age_s", float),
                ("final_position_tolerance_rad", float),
                ("plan_start_tolerance_rad", float),
            )
        })
