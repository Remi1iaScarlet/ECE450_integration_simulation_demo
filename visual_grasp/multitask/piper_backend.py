"""Fail-closed Piper continuous joint-waypoint execution backend.

Importing this module and constructing its classes are side-effect free.  The
Piper SDK is loaded and its interface object is constructed only by the
explicit :meth:`PiperSdkAdapter.connect` call.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Protocol, Sequence, runtime_checkable

from .control import MotionSafetyError, TrajectoryExecutionConfig
from .motion_plan import MotionPlan


# Conservative intersection of this repository's PiperArm limits and the
# current Piper SDK limits.  Execution never clamps: an invalid plan is rejected.
PIPER_JOINT_LIMITS_RAD = tuple(
    (math.radians(lower), math.radians(upper))
    for lower, upper in (
        (-150.0, 150.0),
        (0.0, 180.0),
        (-170.0, 0.0),
        (-100.0, 100.0),
        (-70.0, 70.0),
        (-100.0, 100.0),
    )
)


class PiperDependencyError(RuntimeError):
    """The optional SDK is missing or does not expose the required safe API."""


def _attach_cleanup_error(primary_error: BaseException, context: str,
                          cleanup_error: BaseException) -> str:
    """Keep the primary exception while exposing best-effort cleanup failure."""
    message = f"{context}: {cleanup_error}"
    errors = list(getattr(primary_error, "cleanup_errors", ()))
    errors.append(message)
    try:
        primary_error.cleanup_errors = tuple(errors)
    except Exception:
        pass
    if isinstance(primary_error, MotionSafetyError):
        primary_error.details.setdefault("cleanup_errors", []).append(message)
    add_note = getattr(primary_error, "add_note", None)
    if callable(add_note):
        add_note(message)
    return message


_AUTHORIZATION_SEAL = object()


class HardwareAuthorization:
    """Opaque capability required at every real-hardware control boundary."""

    __slots__ = ("_seal",)

    def __init__(self, seal):
        if seal is not _AUTHORIZATION_SEAL:
            raise TypeError(
                "hardware authorization capabilities can only be issued by the "
                "dual-gate authorization function")
        self._seal = seal


def issue_hardware_authorization(
        *, armed: bool, allow_hardware: bool) -> HardwareAuthorization:
    """Issue an opaque capability only after both strict opt-in gates pass."""
    if armed is not True or allow_hardware is not True:
        raise MotionSafetyError(
            "HARDWARE_NOT_AUTHORIZED",
            "hardware authorization requires armed=true and allow_hardware=true")
    return HardwareAuthorization(_AUTHORIZATION_SEAL)


def _authorization_is_valid(value) -> bool:
    return (isinstance(value, HardwareAuthorization) and
            value._seal is _AUTHORIZATION_SEAL)


@dataclass(frozen=True)
class AdapterCapabilities:
    per_pair_freshness: bool
    freshness_limitation: str


@dataclass(frozen=True)
class RobotState:
    joint_positions_rad: tuple[float, ...]
    timestamp_s: float
    connected: bool
    enabled: bool
    faulted: bool = False
    stopped: bool = False
    status_code: str = "UNKNOWN"
    per_pair_freshness: bool = False
    freshness_evidence: tuple[str, ...] = ()


@dataclass(frozen=True)
class JointTrajectoryReport:
    phase: str
    command_count: int
    max_following_error_rad: float
    final_following_error_rad: float
    duration_s: float


@dataclass(frozen=True)
class MotionPlanExecutionReport:
    segment_reports: tuple[JointTrajectoryReport, ...]


@runtime_checkable
class PiperAdapter(Protocol):
    """SI-unit seam implemented by the real SDK adapter and offline fakes."""

    def connect(self) -> None: ...
    def enable(self) -> None: ...
    def read_state(self) -> RobotState | None: ...
    def command_joint_positions(
            self, joints_rad: Sequence[float], speed_percent: int) -> None: ...
    def command_gripper(self, opening_m: float, effort: int) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class RobotControlBackend(Protocol):
    def connect(self) -> None: ...
    def enable(self) -> None: ...
    def execute_motion_plan(self, motion_plan, config=None): ...
    def execute_grasp_plan(self, motion_plan, config=None, **kwargs): ...
    def command_gripper(self, opening_m: float, effort: int = 1000) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class PiperHardwareBackend:
    """Execute validated joint polylines using fresh Piper feedback as truth."""

    def __init__(
            self, adapter: PiperAdapter,
            *,
            clock: Callable[[], float] = time.monotonic,
            sleeper: Callable[[float], None] = time.sleep,
            joint_limits_rad: Sequence[Sequence[float]] = PIPER_JOINT_LIMITS_RAD,
            max_joint_velocity_rad_s: float = 0.8,
            max_sdk_speed_percent: int = 20,
            gripper_range_m: tuple[float, float] = (0.0, 0.07),
            authorization: HardwareAuthorization | None = None,
            default_config: TrajectoryExecutionConfig | None = None):
        self.adapter = adapter
        self._clock = clock
        self._sleep = sleeper
        self.joint_limits_rad = tuple(
            (float(bounds[0]), float(bounds[1])) for bounds in joint_limits_rad)
        self.max_joint_velocity_rad_s = float(max_joint_velocity_rad_s)
        self.max_sdk_speed_percent = int(max_sdk_speed_percent)
        self.gripper_range_m = tuple(float(value) for value in gripper_range_m)
        self._authorization = authorization
        self.default_config = default_config or TrajectoryExecutionConfig()
        self._connected = False
        self._enabled = False
        self._closed = False
        self._validate_backend_limits()

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def enabled(self) -> bool:
        return self._enabled

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except Exception as cleanup_error:
            if exc is None:
                raise
            if hasattr(exc, "add_note"):
                exc.add_note(f"Piper backend cleanup failed: {cleanup_error}")
        return False

    def connect(self) -> None:
        self._require_authorization()
        if self._closed:
            raise MotionSafetyError("BACKEND_CLOSED", "closed backend cannot reconnect")
        if self._connected:
            return
        try:
            self.adapter.connect()
        except Exception as exc:
            cleanup_error_message = None
            cleanup_failure = None
            try:
                self.adapter.close()
            except Exception as cleanup_error:
                cleanup_failure = cleanup_error
                cleanup_error_message = _attach_cleanup_error(
                    exc, "Piper adapter cleanup after connect failure failed",
                    cleanup_error)
            details = {"cause_type": type(exc).__name__}
            inherited_cleanup = tuple(getattr(exc, "cleanup_errors", ()))
            if inherited_cleanup:
                details["adapter_cleanup_errors"] = list(inherited_cleanup)
            if cleanup_error_message is not None:
                details["cleanup_error"] = cleanup_error_message
            error = MotionSafetyError(
                "CONNECT_FAILED", f"Piper connection failed: {exc}",
                details)
            if cleanup_error_message is not None:
                _attach_cleanup_error(
                    error, "Piper backend connect cleanup failed", cleanup_failure)
            raise error from exc
        self._connected = True

    def enable(self) -> None:
        self._require_authorization()
        if not self._connected:
            raise MotionSafetyError("NOT_CONNECTED", "connect must succeed before enable")
        if self._enabled:
            return
        try:
            self.adapter.enable()
            state = self._read_fresh_state(self.default_config, require_enabled=False)
            if not state.enabled:
                raise MotionSafetyError(
                    "ENABLE_NOT_CONFIRMED",
                    "enable command was not confirmed by fresh motor feedback")
        except MotionSafetyError as error:
            self._rollback_failed_enable(error)
            raise
        except Exception as exc:
            error = MotionSafetyError(
                "ENABLE_FAILED", f"Piper enable failed: {exc}",
                {"cause_type": type(exc).__name__})
            self._rollback_failed_enable(error)
            raise error from exc
        self._enabled = True

    def execute_joint_waypoints(
            self, waypoints: Sequence[Sequence[float]],
            config: TrajectoryExecutionConfig | None = None,
            *, phase: str = "UNSPECIFIED") -> JointTrajectoryReport:
        config = config or self.default_config
        points = self._validate_trajectory(waypoints)
        self._validate_execution_config(config)
        self._validate_nominal_durations(points, config, phase=str(phase))
        self._require_authorization()
        start_time = self._clock()
        command_count = 0
        max_error = 0.0
        final_error = math.inf
        try:
            state = self._require_ready_state(config)
            previous_state = state
            start_error = max(abs(actual - expected) for actual, expected in zip(
                state.joint_positions_rad, points[0]))
            if start_error > config.plan_start_tolerance_rad:
                raise MotionSafetyError(
                    "START_STATE_MISMATCH",
                    "current feedback does not match the trajectory start waypoint",
                    {"error_rad": start_error,
                     "tolerance_rad": config.plan_start_tolerance_rad,
                     "phase": phase})

            speed_percent = max(1, min(self.max_sdk_speed_percent, math.ceil(
                config.max_joint_velocity_rad_s /
                self.max_joint_velocity_rad_s * self.max_sdk_speed_percent)))
            for waypoint_index, (start, end) in enumerate(zip(points, points[1:]), 1):
                distance = max(abs(target - origin)
                               for origin, target in zip(start, end))
                steps = max(1, math.ceil(
                    distance /
                    (config.max_joint_velocity_rad_s * config.control_period_s)))
                deadline = self._clock() + config.waypoint_timeout_s
                for sample_index in range(1, steps + 1):
                    if self._clock() >= deadline:
                        raise self._waypoint_timeout(
                            phase, waypoint_index, final_error, config)
                    alpha = sample_index / steps
                    target = tuple(origin + alpha * (finish - origin)
                                   for origin, finish in zip(start, end))
                    try:
                        self.adapter.command_joint_positions(target, speed_percent)
                    except Exception as exc:
                        raise MotionSafetyError(
                            "COMMAND_FAILED", f"joint command failed: {exc}",
                            {"phase": phase, "waypoint_index": waypoint_index,
                             "sample_index": sample_index,
                             "cause_type": type(exc).__name__}) from exc
                    command_count += 1
                    self._sleep(config.control_period_s)
                    state = self._read_fresh_state(
                        config, require_enabled=True, newer_than=previous_state)
                    if self._clock() > deadline:
                        raise self._waypoint_timeout(
                            phase, waypoint_index, final_error, config)
                    self._check_observed_speed(previous_state, state, phase,
                                               waypoint_index, sample_index)
                    previous_state = state
                    final_error = max(abs(actual - expected)
                                      for actual, expected in zip(
                                          state.joint_positions_rad, target))
                    max_error = max(max_error, final_error)
                    if final_error > config.max_following_error_rad:
                        raise MotionSafetyError(
                            "FOLLOWING_ERROR",
                            "joint following error exceeded the configured limit",
                            {"phase": phase, "waypoint_index": waypoint_index,
                             "sample_index": sample_index,
                             "error_rad": final_error,
                             "limit_rad": config.max_following_error_rad})

                while final_error > config.final_position_tolerance_rad:
                    if self._clock() >= deadline:
                        raise self._waypoint_timeout(
                            phase, waypoint_index, final_error, config)
                    try:
                        self.adapter.command_joint_positions(end, speed_percent)
                    except Exception as exc:
                        raise MotionSafetyError(
                            "COMMAND_FAILED", f"joint hold command failed: {exc}",
                            {"phase": phase, "waypoint_index": waypoint_index,
                             "cause_type": type(exc).__name__}) from exc
                    command_count += 1
                    self._sleep(config.control_period_s)
                    state = self._read_fresh_state(
                        config, require_enabled=True, newer_than=previous_state)
                    if self._clock() > deadline:
                        raise self._waypoint_timeout(
                            phase, waypoint_index, final_error, config)
                    self._check_observed_speed(previous_state, state, phase,
                                               waypoint_index, command_count)
                    previous_state = state
                    final_error = max(abs(actual - expected)
                                      for actual, expected in zip(
                                          state.joint_positions_rad, end))
                    max_error = max(max_error, final_error)
                    if final_error > config.max_following_error_rad:
                        raise MotionSafetyError(
                            "FOLLOWING_ERROR",
                            "joint following error exceeded the configured limit",
                            {"phase": phase, "waypoint_index": waypoint_index,
                             "error_rad": final_error,
                             "limit_rad": config.max_following_error_rad})
        except MotionSafetyError as exc:
            self._attempt_stop(exc)
            raise
        except Exception as exc:
            error = MotionSafetyError(
                "EXECUTION_FAILED", f"trajectory execution failed: {exc}",
                {"phase": phase, "cause_type": type(exc).__name__})
            self._attempt_stop(error)
            raise error from exc

        return JointTrajectoryReport(
            phase=str(phase),
            command_count=command_count,
            max_following_error_rad=float(max_error),
            final_following_error_rad=float(final_error),
            duration_s=float(self._clock() - start_time),
        )

    # SimWorld-compatible name.  Hardware arrival is feedback-based, so the
    # simulation-only settle/clearance arguments are deliberately not accepted.
    follow_joint_waypoints = execute_joint_waypoints

    def execute_motion_plan(
            self, motion_plan: MotionPlan,
            config: TrajectoryExecutionConfig | None = None
            ) -> MotionPlanExecutionReport:
        config = config or self.default_config
        segments, _ = self._prepare_motion_plan(motion_plan, config)
        return self._execute_prepared_segments(segments, config)

    def execute_grasp_plan(
            self, motion_plan: MotionPlan,
            config: TrajectoryExecutionConfig | None = None,
            *, gripper_closed_m: float = 0.0,
            gripper_effort: int = 1000) -> MotionPlanExecutionReport:
        """Execute an explicitly grasping plan, closing once before first LIFT."""
        config = config or self.default_config
        segments, _ = self._prepare_motion_plan(motion_plan, config)
        if not any(str(segment.phase) == "LIFT" for segment in segments):
            raise MotionSafetyError(
                "GRASP_PLAN_MISSING_LIFT",
                "explicit grasp execution requires a LIFT segment")
        self._validate_gripper_opening(gripper_closed_m)
        return self._execute_prepared_segments(
            segments, config, close_before_lift=True,
            gripper_closed_m=gripper_closed_m,
            gripper_effort=gripper_effort)

    def _prepare_motion_plan(self, motion_plan, config):
        segments = list(getattr(motion_plan, "segments", ()))
        if not segments:
            raise MotionSafetyError("EMPTY_MOTION_PLAN", "motion plan has no segments")
        self._validate_execution_config(config)
        # Validate every waypoint before the first hardware command.
        validated_segments = []
        for segment in segments:
            points = self._validate_trajectory(
                [waypoint.q for waypoint in getattr(segment, "waypoints", ())])
            self._validate_nominal_durations(
                points, config, phase=str(segment.phase))
            validated_segments.append(points)
        plan_start = self._validate_joint_vector(
            getattr(motion_plan, "start_q", ()), source="motion_plan.start_q",
            check_limits=True)
        start_gap = max(abs(a - b) for a, b in zip(
            plan_start, validated_segments[0][0]))
        if start_gap > config.plan_start_tolerance_rad:
            raise MotionSafetyError(
                "PLAN_START_MISMATCH",
                "motion_plan.start_q does not match its first waypoint",
                {"error_rad": start_gap,
                 "tolerance_rad": config.plan_start_tolerance_rad})
        for index in range(1, len(validated_segments)):
            seam_gap = max(abs(a - b) for a, b in zip(
                validated_segments[index - 1][-1],
                validated_segments[index][0]))
            if seam_gap > config.plan_start_tolerance_rad:
                raise MotionSafetyError(
                    "PLAN_SEGMENT_DISCONTINUITY",
                    "adjacent motion plan segments do not share an endpoint",
                    {"segment_index": index, "error_rad": seam_gap,
                     "tolerance_rad": config.plan_start_tolerance_rad})
        return segments, validated_segments

    def _execute_prepared_segments(
            self, segments, config, *, close_before_lift=False,
            gripper_closed_m=0.0, gripper_effort=1000):
        reports = []
        gripper_closed = False
        for segment in segments:
            if (close_before_lift and not gripper_closed and
                    str(segment.phase) == "LIFT"):
                self.command_gripper(gripper_closed_m, gripper_effort)
                gripper_closed = True
            reports.append(self.execute_joint_waypoints(
                [waypoint.q for waypoint in segment.waypoints], config,
                phase=str(segment.phase)))
        return MotionPlanExecutionReport(tuple(reports))

    def command_gripper(self, opening_m: float, effort: int = 1000) -> None:
        self._require_authorization()
        opening = self._validate_gripper_opening(opening_m)
        try:
            self._require_ready_state(self.default_config)
            self.adapter.command_gripper(opening, int(effort))
        except MotionSafetyError as exc:
            self._attempt_stop(exc)
            raise
        except Exception as exc:
            error = MotionSafetyError(
                "GRIPPER_COMMAND_FAILED", f"gripper command failed: {exc}",
                {"cause_type": type(exc).__name__})
            self._attempt_stop(error)
            raise error from exc

    def stop(self) -> None:
        self._require_authorization()
        if not self._connected:
            self._enabled = False
            return
        try:
            self.adapter.stop()
        finally:
            self._enabled = False

    def close(self) -> None:
        if self._closed:
            return
        if self._connected:
            self._require_authorization()
        stop_error = None
        try:
            self.stop()
        except Exception as exc:
            stop_error = exc
        try:
            if self._connected:
                self.adapter.close()
        finally:
            self._connected = False
            self._enabled = False
            self._closed = True
        if stop_error is not None:
            raise stop_error

    def _attempt_stop(self, original_error: MotionSafetyError) -> None:
        try:
            self.stop()
        except Exception as stop_error:
            original_error.details["stop_error"] = str(stop_error)

    def _rollback_failed_enable(self, original_error: MotionSafetyError) -> None:
        """Best-effort emergency stop and disconnect after ambiguous enable."""
        try:
            self.close()
        except Exception as rollback_error:
            original_error.details["rollback_error"] = str(rollback_error)

    def _require_authorization(self) -> None:
        if not _authorization_is_valid(self._authorization):
            raise MotionSafetyError(
                "HARDWARE_NOT_AUTHORIZED",
                "Piper backend requires an explicit hardware authorization capability")

    def _require_ready_state(
            self, config: TrajectoryExecutionConfig) -> RobotState:
        if not self._connected:
            raise MotionSafetyError("NOT_CONNECTED", "backend is not connected")
        if not self._enabled:
            raise MotionSafetyError("NOT_ENABLED", "backend is not enabled")
        return self._read_fresh_state(config, require_enabled=True)

    def _read_fresh_state(
            self, config: TrajectoryExecutionConfig,
            *, require_enabled: bool,
            newer_than: RobotState | None = None) -> RobotState:
        deadline = self._clock() + config.feedback_timeout_s
        state = None
        last_nonadvancing = None
        while self._clock() < deadline:
            try:
                state = self.adapter.read_state()
            except MotionSafetyError:
                raise
            except Exception as exc:
                raise MotionSafetyError(
                    "FEEDBACK_FAILED", f"failed to read Piper feedback: {exc}",
                    {"cause_type": type(exc).__name__}) from exc
            if state is None:
                self._sleep(min(config.control_period_s, 0.01))
                continue
            self._validate_feedback_state(
                state, config, require_enabled=require_enabled,
                check_age=False)
            timestamp_not_advanced = (
                newer_than is not None and
                float(state.timestamp_s) <= float(newer_than.timestamp_s))
            if not timestamp_not_advanced:
                self._validate_feedback_state(
                    state, config, require_enabled=require_enabled,
                    check_age=True)
            if timestamp_not_advanced:
                if tuple(state.joint_positions_rad) != tuple(
                        newer_than.joint_positions_rad):
                    raise MotionSafetyError(
                        "FEEDBACK_TIMESTAMP_POSITION_CHANGE",
                        "joint positions changed without an advancing feedback timestamp",
                        {"previous_timestamp_s": newer_than.timestamp_s,
                         "current_timestamp_s": state.timestamp_s})
                last_nonadvancing = state
                state = None
                self._sleep(min(config.control_period_s, 0.01))
                continue
            return state
        if newer_than is not None and last_nonadvancing is not None:
            raise MotionSafetyError(
                "FEEDBACK_TIMESTAMP_TIMEOUT",
                "Piper feedback timestamp did not advance before timeout",
                {"previous_timestamp_s": newer_than.timestamp_s,
                 "last_timestamp_s": last_nonadvancing.timestamp_s,
                 "timeout_s": config.feedback_timeout_s})
        if state is None:
            raise MotionSafetyError(
                "FEEDBACK_TIMEOUT", "no Piper joint feedback before timeout",
                {"timeout_s": config.feedback_timeout_s})
        raise MotionSafetyError(
            "FEEDBACK_TIMEOUT", "no Piper joint feedback before timeout",
            {"timeout_s": config.feedback_timeout_s})

    def _validate_feedback_state(
            self, state: RobotState, config: TrajectoryExecutionConfig,
            *, require_enabled: bool, check_age: bool = True) -> None:
        if not state.connected:
            raise MotionSafetyError("NOT_CONNECTED", "feedback reports disconnected")
        if state.stopped:
            raise MotionSafetyError(
                "ROBOT_STOPPED", "feedback reports stopped state",
                {"status_code": state.status_code})
        if state.faulted:
            raise MotionSafetyError(
                "ROBOT_FAULT", "feedback reports a robot fault",
                {"status_code": state.status_code})
        if require_enabled and not state.enabled:
            raise MotionSafetyError("NOT_ENABLED", "feedback reports disabled motors")
        if not state.per_pair_freshness:
            raise MotionSafetyError(
                "JOINT_FRESHNESS_UNPROVEN",
                "feedback cannot prove per-pair freshness for all six joints",
                {"freshness_evidence": list(state.freshness_evidence)})
        timestamp = float(state.timestamp_s)
        age = self._clock() - timestamp
        if not math.isfinite(timestamp) or age < -1e-6:
            raise MotionSafetyError(
                "STATE_TIMESTAMP", "feedback timestamp is invalid or in the future",
                {"timestamp_s": timestamp, "now_s": self._clock()})
        if check_age and age > config.state_max_age_s:
            raise MotionSafetyError(
                "STATE_STALE", "Piper feedback is stale",
                {"age_s": age, "max_age_s": config.state_max_age_s})
        self._validate_joint_vector(
            state.joint_positions_rad, source="feedback", check_limits=True)

    def _validate_trajectory(
            self, waypoints: Sequence[Sequence[float]]) -> tuple[tuple[float, ...], ...]:
        points = tuple(
            self._validate_joint_vector(point, source=f"waypoint[{index}]",
                                        check_limits=True)
            for index, point in enumerate(waypoints))
        if len(points) < 2:
            raise MotionSafetyError(
                "INVALID_TRAJECTORY", "joint trajectory requires at least two waypoints")
        return points

    def _validate_joint_vector(
            self, values: Sequence[float], *, source: str,
            check_limits: bool) -> tuple[float, ...]:
        try:
            point = tuple(float(value) for value in values)
        except (TypeError, ValueError) as exc:
            raise MotionSafetyError(
                "NON_FINITE_JOINT", f"{source} contains a non-numeric joint") from exc
        if len(point) != 6:
            raise MotionSafetyError(
                "INVALID_JOINT_COUNT", f"{source} must contain exactly 6 joints",
                {"count": len(point)})
        if not all(math.isfinite(value) for value in point):
            raise MotionSafetyError(
                "NON_FINITE_JOINT", f"{source} contains NaN or infinity")
        if check_limits:
            for index, (value, bounds) in enumerate(
                    zip(point, self.joint_limits_rad), 1):
                if not bounds[0] <= value <= bounds[1]:
                    raise MotionSafetyError(
                        "JOINT_LIMIT", f"{source} joint {index} exceeds its limit",
                        {"joint": index, "value_rad": value,
                         "lower_rad": bounds[0], "upper_rad": bounds[1]})
        return point

    def _validate_execution_config(self, config: TrajectoryExecutionConfig) -> None:
        numeric_positive = {
            "max_joint_velocity_rad_s": config.max_joint_velocity_rad_s,
            "max_following_error_rad": config.max_following_error_rad,
            "control_period_s": config.control_period_s,
            "feedback_timeout_s": config.feedback_timeout_s,
            "waypoint_timeout_s": config.waypoint_timeout_s,
            "state_max_age_s": config.state_max_age_s,
            "final_position_tolerance_rad": config.final_position_tolerance_rad,
            "plan_start_tolerance_rad": config.plan_start_tolerance_rad,
        }
        invalid = [name for name, value in numeric_positive.items()
                   if not math.isfinite(float(value)) or float(value) <= 0]
        if invalid:
            raise MotionSafetyError(
                "INVALID_EXECUTION_CONFIG",
                "trajectory execution limits must be finite and positive",
                {"invalid_fields": invalid})
        if config.max_joint_velocity_rad_s > self.max_joint_velocity_rad_s:
            raise MotionSafetyError(
                "SPEED_LIMIT", "requested joint velocity exceeds backend hard limit",
                {"requested_rad_s": config.max_joint_velocity_rad_s,
                 "hard_limit_rad_s": self.max_joint_velocity_rad_s})

    def _validate_nominal_durations(self, points, config, *, phase: str) -> None:
        for waypoint_index, (start, end) in enumerate(zip(points, points[1:]), 1):
            distance = max(abs(target - origin)
                           for origin, target in zip(start, end))
            steps = max(1, math.ceil(
                distance /
                (config.max_joint_velocity_rad_s * config.control_period_s)))
            nominal_duration = steps * config.control_period_s
            if nominal_duration > config.waypoint_timeout_s + 1e-12:
                raise MotionSafetyError(
                    "NOMINAL_WAYPOINT_TIMEOUT",
                    "nominal velocity-limited waypoint duration exceeds timeout",
                    {"phase": phase, "waypoint_index": waypoint_index,
                     "nominal_duration_s": nominal_duration,
                     "timeout_s": config.waypoint_timeout_s})

    def _validate_gripper_opening(self, opening_m: float) -> float:
        try:
            opening = float(opening_m)
        except (TypeError, ValueError) as exc:
            raise MotionSafetyError(
                "GRIPPER_RANGE", "gripper opening must be a finite number") from exc
        lower, upper = self.gripper_range_m
        if not math.isfinite(opening) or not lower <= opening <= upper:
            raise MotionSafetyError(
                "GRIPPER_RANGE",
                f"gripper opening must be within [{lower}, {upper}] m",
                {"opening_m": opening, "lower_m": lower, "upper_m": upper})
        return opening

    def _check_observed_speed(
            self, previous: RobotState, current: RobotState,
            phase: str, waypoint_index: int, sample_index: int) -> None:
        elapsed = current.timestamp_s - previous.timestamp_s
        if elapsed <= 0:
            return
        observed = max(abs(after - before) for before, after in zip(
            previous.joint_positions_rad, current.joint_positions_rad)) / elapsed
        if observed > self.max_joint_velocity_rad_s + 1e-9:
            raise MotionSafetyError(
                "OBSERVED_OVERSPEED",
                "joint feedback velocity exceeded the backend hard limit",
                {"phase": phase, "waypoint_index": waypoint_index,
                 "sample_index": sample_index,
                 "observed_rad_s": observed,
                 "hard_limit_rad_s": self.max_joint_velocity_rad_s})

    def _validate_backend_limits(self) -> None:
        if len(self.joint_limits_rad) != 6:
            raise ValueError("Piper backend requires exactly six joint limits")
        for lower, upper in self.joint_limits_rad:
            if not all(math.isfinite(value) for value in (lower, upper)) or lower > upper:
                raise ValueError("invalid Piper joint limit bounds")
        if (not math.isfinite(self.max_joint_velocity_rad_s) or
                self.max_joint_velocity_rad_s <= 0):
            raise ValueError("max_joint_velocity_rad_s must be finite and positive")
        if not 1 <= self.max_sdk_speed_percent <= 100:
            raise ValueError("max_sdk_speed_percent must be within [1, 100]")
        if (len(self.gripper_range_m) != 2 or
                not all(math.isfinite(value) for value in self.gripper_range_m) or
                self.gripper_range_m[0] < 0.0 or
                self.gripper_range_m[0] > self.gripper_range_m[1] or
                self.gripper_range_m[1] > 0.07):
            raise ValueError(
                "gripper range must stay within the Piper SDK [0.0, 0.07] m limit")

    @staticmethod
    def _waypoint_timeout(
            phase: str, waypoint_index: int, error: float,
            config: TrajectoryExecutionConfig) -> MotionSafetyError:
        return MotionSafetyError(
            "WAYPOINT_TIMEOUT", "waypoint was not confirmed by feedback before timeout",
            {"phase": phase, "waypoint_index": waypoint_index,
             "error_rad": error,
             "tolerance_rad": config.final_position_tolerance_rad,
             "timeout_s": config.waypoint_timeout_s})


class PiperSdkAdapter:
    """Lazy unit-converting adapter for ``piper_sdk.C_PiperInterface_V2``.

    None of the methods in this class are exercised by the offline test suite;
    runtime use still requires a version/firmware-specific commissioning pass.
    """

    _JOINT_COMMAND_FACTOR = 1000.0 * 180.0 / math.pi

    def __init__(
            self, can_name: str = "can0", *, sdk_factory=None,
            authorization: HardwareAuthorization | None = None,
            pair_timestamp_reader=None,
            max_sdk_speed_percent: int = 20,
            clock: Callable[[], float] = time.monotonic):
        self.can_name = str(can_name)
        self._sdk_factory = sdk_factory
        self._authorization = authorization
        self._pair_timestamp_reader = pair_timestamp_reader
        try:
            speed_cap = float(max_sdk_speed_percent)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "max_sdk_speed_percent must be an integer within [1, 100]") from exc
        if (not math.isfinite(speed_cap) or not speed_cap.is_integer() or
                not 1 <= speed_cap <= 100):
            raise ValueError(
                "max_sdk_speed_percent must be an integer within [1, 100]")
        self.max_sdk_speed_percent = int(speed_cap)
        self._clock = clock
        self._piper = None
        self._connected = False
        self._last_joint_sdk_timestamp = None
        self._last_status_sdk_timestamp = None
        self._last_pair_timestamps = None
        self._last_received_at = None
        self.capabilities = AdapterCapabilities(
            per_pair_freshness=(pair_timestamp_reader is not None),
            freshness_limitation=(
                "configured joint-pair timestamp reader"
                if pair_timestamp_reader is not None else
                "public Piper SDK exposes only aggregate joint feedback time; "
                "three joint-pair freshness values are unavailable"))

    def connect(self) -> None:
        self._require_authorization()
        if self._connected:
            return
        factory = self._sdk_factory or self._load_sdk_factory()
        # SDK construction is intentionally delayed until this explicit action.
        self._piper = factory(
            self.can_name, start_sdk_joint_limit=True,
            start_sdk_gripper_limit=True)
        try:
            self._require_methods(
                "ConnectPort", "EnableArm", "JointCtrl", "GripperCtrl",
                "GetArmJointMsgs", "GetArmStatus", "GetArmEnableStatus",
                "DisconnectPort")
            if not (callable(getattr(self._piper, "ModeCtrl", None)) or
                    callable(getattr(self._piper, "MotionCtrl_2", None))):
                raise PiperDependencyError(
                    "piper_sdk must provide ModeCtrl or MotionCtrl_2")
            if not (callable(getattr(self._piper, "EmergencyStop", None)) or
                    callable(getattr(self._piper, "MotionCtrl_2", None))):
                raise PiperDependencyError(
                    "piper_sdk must provide EmergencyStop or MotionCtrl_2 for stop")
            self._piper.ConnectPort()
        except Exception as exc:
            self._disconnect_constructed_sdk(exc)
            raise
        self._connected = True

    def enable(self) -> None:
        self._require_authorization()
        self._require_connected()
        self._piper.EnableArm(7)

    def read_state(self) -> RobotState:
        self._require_authorization()
        self._require_connected()
        joint_message = self._piper.GetArmJointMsgs()
        status_message = self._piper.GetArmStatus()
        joint_timestamp = self._finite_feedback_value(
            joint_message, "time_stamp", "joint timestamp")
        status_timestamp = self._finite_feedback_value(
            status_message, "time_stamp", "status timestamp")
        joint_hz = self._positive_feedback_value(
            joint_message, "Hz", "joint feedback Hz")
        status_hz = self._positive_feedback_value(
            status_message, "Hz", "status feedback Hz")
        joint_advanced = self._timestamp_advanced(
            joint_timestamp, self._last_joint_sdk_timestamp, "joint")
        status_advanced = self._timestamp_advanced(
            status_timestamp, self._last_status_sdk_timestamp, "status")

        per_pair_freshness = False
        evidence = (
            f"joint_hz={joint_hz:g}", f"status_hz={status_hz:g}",
            "enable_status=GetArmEnableStatus")
        if self._pair_timestamp_reader is not None:
            try:
                pair_timestamps = tuple(float(value) for value in
                                        self._pair_timestamp_reader(joint_message))
            except Exception as exc:
                raise PiperDependencyError(
                    f"joint-pair timestamp reader failed: {exc}") from exc
            if (len(pair_timestamps) != 3 or
                    not all(math.isfinite(value) for value in pair_timestamps)):
                raise PiperDependencyError(
                    "joint-pair freshness evidence must contain three finite timestamps")
            if self._last_pair_timestamps is None:
                pair_advanced = True
            else:
                if any(current < previous for current, previous in zip(
                        pair_timestamps, self._last_pair_timestamps)):
                    raise PiperDependencyError(
                        "joint-pair feedback timestamp moved backwards")
                pair_advanced = all(current > previous for current, previous in zip(
                    pair_timestamps, self._last_pair_timestamps))
            self._last_pair_timestamps = pair_timestamps
            per_pair_freshness = pair_advanced
            evidence += ("three_pair_timestamps",)
        else:
            evidence += (self.capabilities.freshness_limitation,)

        if joint_advanced and status_advanced and per_pair_freshness:
            self._last_received_at = self._clock()
        elif self._last_received_at is None:
            # Timestamp remains usable for diagnostics, but the explicit
            # per-pair flag makes the hardware backend reject this state.
            self._last_received_at = self._clock()
        self._last_joint_sdk_timestamp = joint_timestamp
        self._last_status_sdk_timestamp = status_timestamp
        joints = joint_message.joint_state
        raw = (joints.joint_1, joints.joint_2, joints.joint_3,
               joints.joint_4, joints.joint_5, joints.joint_6)
        positions = tuple(
            math.radians(float(value) * 1e-3) for value in raw)
        raw_enable_status = self._piper.GetArmEnableStatus()
        try:
            enable_status = tuple(bool(value) for value in raw_enable_status)
        except TypeError as exc:
            raise PiperDependencyError(
                "GetArmEnableStatus must return six motor flags") from exc
        if len(enable_status) != 6:
            raise PiperDependencyError(
                "GetArmEnableStatus must return exactly six motor flags")
        status = status_message.arm_status
        status_code = int(status.arm_status)
        return RobotState(
            joint_positions_rad=positions,
            timestamp_s=float(self._last_received_at),
            connected=self._connected,
            enabled=(len(enable_status) == 6 and all(enable_status)),
            faulted=(status_code != 0),
            stopped=(status_code == 1),
            status_code=f"0x{status_code:02X}",
            per_pair_freshness=per_pair_freshness,
            freshness_evidence=evidence,
        )

    def command_joint_positions(
            self, joints_rad: Sequence[float], speed_percent: int) -> None:
        self._require_authorization()
        self._require_connected()
        try:
            command_values = tuple(float(value) for value in joints_rad)
        except (TypeError, ValueError) as exc:
            raise MotionSafetyError(
                "NON_FINITE_JOINT", "adapter joint command must be numeric") from exc
        if len(command_values) != 6:
            raise MotionSafetyError(
                "INVALID_JOINT_COUNT",
                "adapter joint command must contain exactly six joints",
                {"count": len(command_values)})
        if not all(math.isfinite(value) for value in command_values):
            raise MotionSafetyError(
                "NON_FINITE_JOINT",
                "adapter joint command contains NaN or infinity")
        for index, (value, bounds) in enumerate(
                zip(command_values, PIPER_JOINT_LIMITS_RAD), 1):
            if not bounds[0] <= value <= bounds[1]:
                raise MotionSafetyError(
                    "JOINT_LIMIT",
                    f"adapter joint command {index} exceeds its Piper limit",
                    {"joint": index, "value_rad": value,
                     "lower_rad": bounds[0], "upper_rad": bounds[1]})
        try:
            speed_value = float(speed_percent)
        except (TypeError, ValueError) as exc:
            raise MotionSafetyError(
                "SDK_SPEED_LIMIT",
                "SDK speed percent must be an integer within the adapter cap",
                {"hard_cap_percent": self.max_sdk_speed_percent}) from exc
        if (not math.isfinite(speed_value) or not speed_value.is_integer() or
                not 1 <= speed_value <= self.max_sdk_speed_percent):
            raise MotionSafetyError(
                "SDK_SPEED_LIMIT",
                "SDK speed percent exceeds the adapter hard cap",
                {"requested_percent": speed_percent,
                 "hard_cap_percent": self.max_sdk_speed_percent})
        speed = int(speed_value)
        if callable(getattr(self._piper, "ModeCtrl", None)):
            self._piper.ModeCtrl(0x01, 0x01, speed, 0x00)
        else:
            self._piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        command = tuple(int(value * self._JOINT_COMMAND_FACTOR)
                        for value in command_values)
        self._piper.JointCtrl(*command)

    def command_gripper(self, opening_m: float, effort: int) -> None:
        self._require_authorization()
        self._require_connected()
        try:
            opening = float(opening_m)
        except (TypeError, ValueError) as exc:
            raise MotionSafetyError(
                "GRIPPER_RANGE",
                "Piper gripper opening must be a finite number") from exc
        if not math.isfinite(opening) or not 0.0 <= opening <= 0.07:
            raise MotionSafetyError(
                "GRIPPER_RANGE",
                "Piper gripper opening must be within [0.0, 0.07] m",
                {"opening_m": opening, "lower_m": 0.0, "upper_m": 0.07})
        self._piper.GripperCtrl(
            int(round(opening * 1_000_000.0)),
            int(effort), 0x01, 0x00)

    def stop(self) -> None:
        self._require_authorization()
        self._require_connected()
        if hasattr(self._piper, "EmergencyStop"):
            self._piper.EmergencyStop(0x01)
        else:
            # Legacy SDK fallback: enter standby at zero speed.
            self._piper.MotionCtrl_2(0x00, 0x01, 0, 0x00)

    def close(self) -> None:
        self._require_authorization()
        piper = self._piper
        self._piper = None
        self._connected = False
        if piper is not None:
            disconnect = getattr(piper, "DisconnectPort", None)
            if callable(disconnect):
                disconnect()

    @staticmethod
    def _load_sdk_factory():
        try:
            from piper_sdk import C_PiperInterface_V2
        except (ImportError, ModuleNotFoundError) as exc:
            raise PiperDependencyError(
                "real_piper requires the optional 'piper_sdk' package; "
                "dry-run and injected fake backends do not") from exc
        return C_PiperInterface_V2

    def _require_methods(self, *names: str) -> None:
        missing = [name for name in names
                   if not callable(getattr(self._piper, name, None))]
        if missing:
            raise PiperDependencyError(
                "installed piper_sdk is missing required APIs: " +
                ", ".join(missing))

    def _disconnect_constructed_sdk(self, primary_error: BaseException) -> None:
        """Release a constructed SDK object even before connected is true."""
        piper = self._piper
        self._piper = None
        self._connected = False
        if piper is None:
            return
        disconnect = getattr(piper, "DisconnectPort", None)
        if not callable(disconnect):
            return
        try:
            disconnect()
        except Exception as cleanup_error:
            _attach_cleanup_error(
                primary_error, "Piper SDK connect cleanup failed", cleanup_error)

    def _require_connected(self) -> None:
        if not self._connected or self._piper is None:
            raise PiperDependencyError("Piper SDK adapter is not connected")

    def _require_authorization(self) -> None:
        if not _authorization_is_valid(self._authorization):
            raise PiperDependencyError(
                "Piper SDK hardware path requires an authorization capability")

    @staticmethod
    def _finite_feedback_value(message, name: str, description: str) -> float:
        try:
            value = float(getattr(message, name))
        except (AttributeError, TypeError, ValueError) as exc:
            raise PiperDependencyError(f"Piper {description} is unavailable") from exc
        if not math.isfinite(value):
            raise PiperDependencyError(f"Piper {description} is invalid")
        return value

    @classmethod
    def _positive_feedback_value(cls, message, name: str, description: str) -> float:
        value = cls._finite_feedback_value(message, name, description)
        if value <= 0:
            raise PiperDependencyError(f"Piper {description} must be positive")
        return value

    @staticmethod
    def _timestamp_advanced(current: float, previous, source: str) -> bool:
        if previous is None:
            return True
        if current < previous:
            raise PiperDependencyError(
                f"Piper {source} feedback timestamp moved backwards")
        return current > previous


def create_default_piper_backend(
        command=None, *, can_name: str = "can0",
        authorization: HardwareAuthorization | None = None) -> PiperHardwareBackend:
    """Construct a side-effect-free default backend for bridge injection."""
    adapter = PiperSdkAdapter(
        can_name=can_name, authorization=authorization)
    return PiperHardwareBackend(adapter, authorization=authorization)
