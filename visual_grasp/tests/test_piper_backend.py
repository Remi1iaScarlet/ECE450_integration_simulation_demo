import math
import subprocess
import sys
import unittest
from types import SimpleNamespace

import numpy as np


class FakeClock:
    def __init__(self, start=100.0):
        self.now = float(start)

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += float(seconds)


class FakeAdapter:
    """Offline adapter: no Piper SDK/CAN method exists on this fake."""

    def __init__(self, clock):
        self.clock = clock
        self.events = []
        self.connected = False
        self.enabled = False
        self.current = [0.0] * 6
        self.timestamp_override = None
        self.feedback_none = False
        self.follow_commands = True
        self.faulted = False
        self.stopped = False
        self.raise_on_command = None
        self.raise_on_enable = None
        self.raise_on_stop = None
        self.feedback_scale = 1.0
        self.enable_confirmed = True
        self.freeze_timestamp = False
        self._frozen_timestamp = None
        self.per_pair_freshness = True
        self.raise_when_joint0_at_least = None

    def connect(self):
        self.events.append(("connect",))
        self.connected = True

    def enable(self):
        self.events.append(("enable",))
        if self.raise_on_enable is not None:
            raise self.raise_on_enable
        self.enabled = self.enable_confirmed

    def read_state(self):
        if self.feedback_none:
            return None
        from multitask.piper_backend import RobotState
        if self.freeze_timestamp:
            if self._frozen_timestamp is None:
                self._frozen_timestamp = self.clock.monotonic()
            timestamp = self._frozen_timestamp
        else:
            timestamp = (self.clock.monotonic() if self.timestamp_override is None
                         else self.timestamp_override)
        return RobotState(
            joint_positions_rad=tuple(self.current),
            timestamp_s=timestamp,
            connected=self.connected,
            enabled=self.enabled,
            faulted=self.faulted,
            stopped=self.stopped,
            status_code=("FAULT" if self.faulted else "OK"),
            per_pair_freshness=self.per_pair_freshness,
            freshness_evidence=("offline_fake",),
        )

    def command_joint_positions(self, joints_rad, speed_percent):
        if self.raise_on_command is not None:
            raise self.raise_on_command
        target = tuple(float(value) for value in joints_rad)
        if (self.raise_when_joint0_at_least is not None and
                target[0] >= self.raise_when_joint0_at_least):
            raise RuntimeError("scripted lift failure")
        self.events.append(("joint", target, speed_percent))
        if self.follow_commands:
            self.current = [
                current + self.feedback_scale * (desired - current)
                for current, desired in zip(self.current, target)
            ]

    def command_gripper(self, opening_m, effort):
        self.events.append(("gripper", float(opening_m), int(effort)))

    def stop(self):
        self.events.append(("stop",))
        if self.raise_on_stop is not None:
            raise self.raise_on_stop
        self.stopped = True
        self.enabled = False

    def close(self):
        self.events.append(("close",))
        self.connected = False


def make_backend(clock=None, *, authorized=True, **backend_kwargs):
    from multitask.piper_backend import (
        PiperHardwareBackend,
        issue_hardware_authorization,
    )
    clock = clock or FakeClock()
    adapter = FakeAdapter(clock)
    authorization = (issue_hardware_authorization(
        armed=True, allow_hardware=True) if authorized else None)
    backend = PiperHardwareBackend(
        adapter,
        authorization=authorization,
        clock=clock.monotonic,
        sleeper=clock.sleep,
        **backend_kwargs,
    )
    return backend, adapter, clock


def actual_motion_plan(values=(0.0, 0.1, 0.2, 0.3)):
    from multitask.motion_plan import JointWaypoint, MotionPlan, MotionSegment
    phases = ("TRANSIT", "FINAL_APPROACH", "LIFT")
    names = ("start_to_pre", "pre_to_grasp", "grasp_to_lift")
    points = [np.array([value, 0, 0, 0, 0, 0], dtype=float)
              for value in values]
    segments = [
        MotionSegment(names[index], phases[index], [
            JointWaypoint(f"p{index}", points[index]),
            JointWaypoint(f"p{index + 1}", points[index + 1]),
        ])
        for index in range(3)
    ]
    return MotionPlan(points[0].copy(), segments)


class OvershootClock(FakeClock):
    def sleep(self, seconds):
        self.now += float(seconds) + 0.5


class SdkStub:
    def __init__(self):
        self.events = []
        self.joint_timestamp = 1.0
        self.status_timestamp = 2.0
        self.joint_hz = 50.0
        self.status_hz = 20.0
        self.pair_timestamps = (1.0, 1.0, 1.0)
        self.enabled = [True] * 6
        self.status_code = 0

    def ConnectPort(self): self.events.append(("ConnectPort",))
    def EnableArm(self, value): self.events.append(("EnableArm", value))
    def JointCtrl(self, *values): self.events.append(("JointCtrl",) + values)
    def GripperCtrl(self, *values): self.events.append(("GripperCtrl",) + values)
    def ModeCtrl(self, *values): self.events.append(("ModeCtrl",) + values)
    def EmergencyStop(self, value): self.events.append(("EmergencyStop", value))
    def DisconnectPort(self): self.events.append(("DisconnectPort",))
    def GetArmEnableStatus(self): return list(self.enabled)

    def GetArmJointMsgs(self):
        joints = SimpleNamespace(
            joint_1=1000, joint_2=2000, joint_3=-3000,
            joint_4=4000, joint_5=5000, joint_6=6000)
        return SimpleNamespace(
            time_stamp=self.joint_timestamp, Hz=self.joint_hz,
            joint_state=joints, pair_timestamps=self.pair_timestamps)

    def GetArmStatus(self):
        return SimpleNamespace(
            time_stamp=self.status_timestamp, Hz=self.status_hz,
            arm_status=SimpleNamespace(arm_status=self.status_code))


class ConnectRaisesSdkStub(SdkStub):
    def ConnectPort(self):
        self.events.append(("ConnectPort",))
        raise RuntimeError("connect boom")


class ConnectAndDisconnectRaiseSdkStub(ConnectRaisesSdkStub):
    def DisconnectPort(self):
        self.events.append(("DisconnectPort",))
        raise RuntimeError("disconnect boom")


def execution_config(**overrides):
    from multitask.control import TrajectoryExecutionConfig
    values = {
        "max_joint_velocity_rad_s": 0.5,
        "max_following_error_rad": 0.08,
        "control_period_s": 0.1,
        "feedback_timeout_s": 0.25,
        "waypoint_timeout_s": 0.5,
        "state_max_age_s": 0.2,
        "final_position_tolerance_rad": 0.01,
    }
    values.update(overrides)
    return TrajectoryExecutionConfig(**values)


class PiperBackendTests(unittest.TestCase):
    def _ready_backend(self, **kwargs):
        backend, adapter, clock = make_backend(**kwargs)
        backend.connect()
        backend.enable()
        return backend, adapter, clock

    def assert_error_code(self, expected, callable_):
        from multitask.control import MotionSafetyError
        with self.assertRaises(MotionSafetyError) as caught:
            callable_()
        self.assertEqual(caught.exception.code, expected)
        self.assertEqual(caught.exception.to_dict()["code"], expected)
        return caught.exception

    def test_import_and_construction_do_not_load_hardware_or_sim_dependencies(self):
        code = (
            "import sys; "
            "from multitask.piper_backend import PiperHardwareBackend, PiperSdkAdapter; "
            "adapter=PiperSdkAdapter(); backend=PiperHardwareBackend(adapter); "
            "assert 'piper_sdk' not in sys.modules; "
            "assert 'mujoco' not in sys.modules; "
            "assert 'rospy' not in sys.modules"
        )
        result = subprocess.run(
            [sys.executable, "-c", code], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_backend_connect_requires_an_explicit_authorization_capability(self):
        backend, adapter, _ = make_backend(authorized=False)
        self.assert_error_code("HARDWARE_NOT_AUTHORIZED", backend.connect)
        self.assertEqual(adapter.events, [])

    def test_authorization_capability_requires_both_strict_gates(self):
        from multitask.control import MotionSafetyError
        from multitask.piper_backend import issue_hardware_authorization
        for armed, allowed in ((False, True), (True, False), (1, True)):
            with self.subTest(armed=armed, allowed=allowed):
                with self.assertRaises(MotionSafetyError) as caught:
                    issue_hardware_authorization(
                        armed=armed, allow_hardware=allowed)
                self.assertEqual(caught.exception.code,
                                 "HARDWARE_NOT_AUTHORIZED")

    def test_execute_requires_explicit_connect_then_enable(self):
        backend, adapter, _ = make_backend()
        points = [[0.0] * 6, [0.1, 0, 0, 0, 0, 0]]
        self.assert_error_code(
            "NOT_CONNECTED",
            lambda: backend.execute_joint_waypoints(points, execution_config()),
        )
        backend.connect()
        self.assert_error_code(
            "NOT_ENABLED",
            lambda: backend.execute_joint_waypoints(points, execution_config()),
        )
        self.assertFalse(any(event[0] == "joint" for event in adapter.events))

    def test_enable_not_confirmed_rolls_back_and_preserves_error(self):
        backend, adapter, _ = make_backend()
        backend.connect()
        adapter.enable_confirmed = False
        error = self.assert_error_code("ENABLE_NOT_CONFIRMED", backend.enable)
        self.assertIn(("stop",), adapter.events)
        self.assertEqual(adapter.events[-1], ("close",))
        self.assertFalse(backend.connected)
        self.assertFalse(backend.enabled)
        self.assertEqual(error.code, "ENABLE_NOT_CONFIRMED")

    def test_partial_enable_exception_rolls_back_and_preserves_error(self):
        backend, adapter, _ = make_backend()
        backend.connect()
        adapter.raise_on_enable = RuntimeError("enable write failed")
        error = self.assert_error_code("ENABLE_FAILED", backend.enable)
        self.assertIn("enable write failed", str(error))
        self.assertIn(("stop",), adapter.events)
        self.assertEqual(adapter.events[-1], ("close",))

    def test_enable_feedback_timeout_rolls_back_and_preserves_timeout(self):
        backend, adapter, _ = make_backend()
        backend.connect()
        adapter.feedback_none = True
        error = self.assert_error_code("FEEDBACK_TIMEOUT", backend.enable)
        self.assertEqual(error.code, "FEEDBACK_TIMEOUT")
        self.assertIn(("stop",), adapter.events)
        self.assertEqual(adapter.events[-1], ("close",))

    def test_continuous_waypoints_are_interpolated_with_bounded_speed(self):
        backend, adapter, _ = self._ready_backend()
        report = backend.execute_joint_waypoints(
            [[0.0] * 6, [0.2, 0, 0, 0, 0, 0]], execution_config())
        commands = [event for event in adapter.events if event[0] == "joint"]
        self.assertEqual(len(commands), 4)
        positions = [[0.0] * 6] + [list(event[1]) for event in commands]
        max_step = 0.5 * 0.1
        self.assertTrue(all(
            max(abs(b - a) for a, b in zip(previous, current)) <= max_step + 1e-12
            for previous, current in zip(positions, positions[1:])))
        self.assertEqual(commands[-1][1], (0.2, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertLessEqual(report.max_following_error_rad, 0.01)

    def test_requested_velocity_above_backend_limit_is_rejected(self):
        backend, adapter, _ = self._ready_backend(max_joint_velocity_rad_s=0.6)
        self.assert_error_code(
            "SPEED_LIMIT",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.1, 0, 0, 0, 0, 0]],
                execution_config(max_joint_velocity_rad_s=0.7)),
        )
        self.assertFalse(any(event[0] == "joint" for event in adapter.events))

    def test_sdk_speed_percentage_is_capped_independently_of_software_velocity(self):
        backend, adapter, _ = self._ready_backend(
            max_joint_velocity_rad_s=0.8,
            max_sdk_speed_percent=20,
        )
        backend.execute_joint_waypoints(
            [[0.0] * 6, [0.04, 0, 0, 0, 0, 0]],
            execution_config(max_joint_velocity_rad_s=0.4),
        )
        speeds = [event[2] for event in adapter.events if event[0] == "joint"]
        self.assertTrue(speeds)
        self.assertEqual(set(speeds), {10})

    def test_observed_feedback_overspeed_stops_immediately(self):
        backend, adapter, _ = self._ready_backend(max_joint_velocity_rad_s=0.6)
        adapter.feedback_scale = 4.0
        self.assert_error_code(
            "OBSERVED_OVERSPEED",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.2, 0, 0, 0, 0, 0]],
                execution_config(max_following_error_rad=0.5)),
        )
        self.assertEqual(
            len([event for event in adapter.events if event[0] == "joint"]), 1)
        self.assertIn(("stop",), adapter.events)

    def test_same_timestamp_with_changed_positions_is_rejected(self):
        backend, adapter, _ = self._ready_backend()
        adapter.freeze_timestamp = True
        adapter._frozen_timestamp = 100.0
        self.assert_error_code(
            "FEEDBACK_TIMESTAMP_POSITION_CHANGE",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.05, 0, 0, 0, 0, 0]], execution_config()),
        )
        self.assertIn(("stop",), adapter.events)

    def test_same_timestamp_without_motion_waits_then_rejects(self):
        backend, adapter, _ = self._ready_backend()
        adapter.freeze_timestamp = True
        adapter._frozen_timestamp = 100.0
        adapter.follow_commands = False
        self.assert_error_code(
            "FEEDBACK_TIMESTAMP_TIMEOUT",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.05, 0, 0, 0, 0, 0]],
                execution_config(max_following_error_rad=0.5)),
        )
        self.assertIn(("stop",), adapter.events)

    def test_invalid_joint_count_nan_and_limit_are_fail_closed(self):
        invalid_cases = (
            ("INVALID_JOINT_COUNT", [[0.0] * 6, [0.0] * 5]),
            ("NON_FINITE_JOINT", [[0.0] * 6, [math.nan, 0, 0, 0, 0, 0]]),
            ("JOINT_LIMIT", [[0.0] * 6, [9.0, 0, 0, 0, 0, 0]]),
        )
        for code, points in invalid_cases:
            with self.subTest(code=code):
                backend, adapter, _ = self._ready_backend()
                self.assert_error_code(
                    code,
                    lambda: backend.execute_joint_waypoints(points, execution_config()),
                )
                self.assertFalse(any(event[0] == "joint" for event in adapter.events))

    def test_stale_feedback_is_rejected_before_motion(self):
        backend, adapter, clock = self._ready_backend()
        adapter.timestamp_override = clock.monotonic() - 1.0
        self.assert_error_code(
            "STATE_STALE",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.1, 0, 0, 0, 0, 0]], execution_config()),
        )
        self.assertFalse(any(event[0] == "joint" for event in adapter.events))

    def test_feedback_timeout_stops_and_does_not_continue(self):
        backend, adapter, _ = self._ready_backend()
        adapter.feedback_none = True
        self.assert_error_code(
            "FEEDBACK_TIMEOUT",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.1, 0, 0, 0, 0, 0]], execution_config()),
        )
        self.assertIn(("stop",), adapter.events)
        self.assertFalse(any(event[0] == "joint" for event in adapter.events))

    def test_following_error_stops_after_first_bad_control_sample(self):
        backend, adapter, _ = self._ready_backend()
        adapter.follow_commands = False
        self.assert_error_code(
            "FOLLOWING_ERROR",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.2, 0, 0, 0, 0, 0]],
                execution_config(max_following_error_rad=0.01)),
        )
        commands = [event for event in adapter.events if event[0] == "joint"]
        self.assertEqual(len(commands), 1)
        self.assertIn(("stop",), adapter.events)

    def test_waypoint_arrival_uses_feedback_not_sleep_duration(self):
        backend, adapter, _ = self._ready_backend()
        adapter.follow_commands = False
        self.assert_error_code(
            "WAYPOINT_TIMEOUT",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.05, 0, 0, 0, 0, 0]],
                execution_config(
                    max_following_error_rad=0.2,
                    waypoint_timeout_s=0.25)),
        )
        self.assertIn(("stop",), adapter.events)

    def test_fault_and_stopped_state_prevent_any_motion(self):
        for attr, code in (("faulted", "ROBOT_FAULT"), ("stopped", "ROBOT_STOPPED")):
            with self.subTest(state=attr):
                backend, adapter, _ = self._ready_backend()
                setattr(adapter, attr, True)
                self.assert_error_code(
                    code,
                    lambda: backend.execute_joint_waypoints(
                        [[0.0] * 6, [0.1, 0, 0, 0, 0, 0]], execution_config()),
                )
                self.assertFalse(any(event[0] == "joint" for event in adapter.events))

    def test_command_exception_attempts_stop_without_masking_original_error(self):
        backend, adapter, _ = self._ready_backend()
        adapter.raise_on_command = RuntimeError("write boom")
        adapter.raise_on_stop = RuntimeError("stop boom")
        error = self.assert_error_code(
            "COMMAND_FAILED",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.1, 0, 0, 0, 0, 0]], execution_config()),
        )
        self.assertIn("write boom", str(error))
        self.assertIn("stop boom", error.details["stop_error"])
        self.assertIn(("stop",), adapter.events)

    def test_context_manager_closes_and_stops_on_exception(self):
        backend, adapter, _ = make_backend()
        with self.assertRaisesRegex(ValueError, "caller failure"):
            with backend:
                backend.connect()
                backend.enable()
                raise ValueError("caller failure")
        self.assertIn(("stop",), adapter.events)
        self.assertEqual(adapter.events[-1], ("close",))

    def test_gripper_has_explicit_state_and_range_guards(self):
        backend, adapter, _ = self._ready_backend()
        for value in (-0.001, 0.071, math.nan):
            with self.subTest(value=value):
                self.assert_error_code(
                    "GRIPPER_RANGE",
                    lambda value=value: backend.command_gripper(value),
                )
        self.assertFalse(any(event[0] == "gripper" for event in adapter.events))
        backend.command_gripper(0.05, effort=900)
        self.assertIn(("gripper", 0.05, 900), adapter.events)

    def test_nominal_waypoint_duration_is_rejected_before_first_command(self):
        backend, adapter, _ = self._ready_backend()
        self.assert_error_code(
            "NOMINAL_WAYPOINT_TIMEOUT",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.2, 0, 0, 0, 0, 0]],
                execution_config(
                    max_joint_velocity_rad_s=0.1,
                    control_period_s=0.1,
                    waypoint_timeout_s=0.5)),
        )
        self.assertFalse(any(event[0] == "joint" for event in adapter.events))

    def test_entire_motion_plan_nominal_duration_is_atomically_prevalidated(self):
        backend, adapter, _ = self._ready_backend()
        plan = actual_motion_plan(values=(0.0, 0.01, 0.02, 0.30))
        self.assert_error_code(
            "NOMINAL_WAYPOINT_TIMEOUT",
            lambda: backend.execute_motion_plan(
                plan, execution_config(
                    max_joint_velocity_rad_s=0.1,
                    waypoint_timeout_s=0.5)),
        )
        self.assertFalse(any(event[0] == "joint" for event in adapter.events))

    def test_deadline_is_checked_after_final_sleep_and_feedback(self):
        backend, adapter, _ = self._ready_backend(clock=OvershootClock())
        self.assert_error_code(
            "WAYPOINT_TIMEOUT",
            lambda: backend.execute_joint_waypoints(
                [[0.0] * 6, [0.05, 0, 0, 0, 0, 0]],
                execution_config(waypoint_timeout_s=0.5)),
        )
        self.assertEqual(
            len([event for event in adapter.events if event[0] == "joint"]), 1)

    def test_motion_plan_segments_reuse_the_existing_waypoint_contract(self):
        backend, adapter, _ = self._ready_backend()
        plan = actual_motion_plan()
        report = backend.execute_motion_plan(plan, execution_config())
        self.assertEqual([row.phase for row in report.segment_reports],
                         ["TRANSIT", "FINAL_APPROACH", "LIFT"])
        commands = [event for event in adapter.events if event[0] == "joint"]
        self.assertEqual(commands[-1][1][0], 0.3)
        self.assertFalse(any(event[0] == "gripper" for event in adapter.events))

    def test_explicit_grasp_execution_closes_once_immediately_before_lift(self):
        backend, adapter, _ = self._ready_backend()
        backend.execute_grasp_plan(
            actual_motion_plan(), execution_config(),
            gripper_closed_m=0.0, gripper_effort=800)
        gripper_indexes = [index for index, event in enumerate(adapter.events)
                           if event[0] == "gripper"]
        self.assertEqual(len(gripper_indexes), 1)
        lift_index = next(
            index for index, event in enumerate(adapter.events)
            if event[0] == "joint" and event[1][0] > 0.2)
        self.assertLess(gripper_indexes[0], lift_index)
        self.assertEqual(adapter.events[gripper_indexes[0]],
                         ("gripper", 0.0, 800))

    def test_grasp_lift_exception_stops_after_single_close(self):
        backend, adapter, _ = self._ready_backend()
        adapter.raise_when_joint0_at_least = 0.25
        self.assert_error_code(
            "COMMAND_FAILED",
            lambda: backend.execute_grasp_plan(
                actual_motion_plan(), execution_config()),
        )
        self.assertEqual(
            len([event for event in adapter.events if event[0] == "gripper"]), 1)
        self.assertIn(("stop",), adapter.events)

    def test_motion_plan_start_q_must_equal_first_waypoint_before_motion(self):
        backend, adapter, _ = self._ready_backend()
        waypoint = lambda name, q: SimpleNamespace(name=name, q=q)
        plan = SimpleNamespace(
            start_q=[0.4, 0, 0, 0, 0, 0],
            segments=[SimpleNamespace(
                name="start_to_pre", phase="TRANSIT",
                waypoints=[waypoint("start", [0.0] * 6),
                           waypoint("pre", [0.1, 0, 0, 0, 0, 0])])],
        )
        self.assert_error_code(
            "PLAN_START_MISMATCH",
            lambda: backend.execute_motion_plan(plan, execution_config()),
        )
        self.assertFalse(any(event[0] == "joint" for event in adapter.events))

    def test_motion_plan_segment_seams_are_atomically_prevalidated(self):
        backend, adapter, _ = self._ready_backend()
        waypoint = lambda name, q: SimpleNamespace(name=name, q=q)
        plan = SimpleNamespace(
            start_q=[0.0] * 6,
            segments=[
                SimpleNamespace(
                    name="first", phase="TRANSIT",
                    waypoints=[waypoint("start", [0.0] * 6),
                               waypoint("pre", [0.1, 0, 0, 0, 0, 0])]),
                SimpleNamespace(
                    name="gap", phase="FINAL_APPROACH",
                    waypoints=[waypoint("wrong_start", [0.3, 0, 0, 0, 0, 0]),
                               waypoint("grasp", [0.4, 0, 0, 0, 0, 0])]),
            ],
        )
        self.assert_error_code(
            "PLAN_SEGMENT_DISCONTINUITY",
            lambda: backend.execute_motion_plan(plan, execution_config()),
        )
        self.assertFalse(any(event[0] == "joint" for event in adapter.events))

    def test_sdk_adapter_requires_authorization_before_constructing_sdk(self):
        from multitask.piper_backend import PiperSdkAdapter
        constructed = []
        adapter = PiperSdkAdapter(sdk_factory=lambda *args, **kwargs: constructed.append(1))
        with self.assertRaisesRegex(Exception, "authorization"):
            adapter.connect()
        self.assertEqual(constructed, [])

    def test_sdk_shaped_stub_maps_units_commands_status_and_disconnect(self):
        from multitask.piper_backend import (
            PiperSdkAdapter,
            issue_hardware_authorization,
        )
        sdk = SdkStub()
        token = issue_hardware_authorization(armed=True, allow_hardware=True)
        adapter = PiperSdkAdapter(
            sdk_factory=lambda *args, **kwargs: sdk,
            authorization=token,
            pair_timestamp_reader=lambda message: message.pair_timestamps,
            clock=lambda: 10.0)
        adapter.connect()
        adapter.enable()
        state = adapter.read_state()
        self.assertAlmostEqual(state.joint_positions_rad[0], math.radians(1.0))
        self.assertEqual(state.status_code, "0x00")
        self.assertTrue(state.enabled)
        self.assertTrue(state.per_pair_freshness)
        adapter.command_joint_positions([
            math.radians(1.0), math.radians(1.0), math.radians(-1.0),
            math.radians(1.0), math.radians(1.0), math.radians(1.0),
        ], 12)
        adapter.command_gripper(0.07, 900)
        adapter.stop()
        adapter.close()
        self.assertIn(("EnableArm", 7), sdk.events)
        self.assertIn(("ModeCtrl", 0x01, 0x01, 12, 0x00), sdk.events)
        self.assertIn(("JointCtrl", 1000, 1000, -1000, 1000, 1000, 1000),
                      sdk.events)
        self.assertIn(("GripperCtrl", 70000, 900, 0x01, 0x00), sdk.events)
        self.assertIn(("EmergencyStop", 0x01), sdk.events)
        self.assertEqual(sdk.events[-1], ("DisconnectPort",))

    def test_sdk_adapter_direct_bypass_rejects_joint_and_speed_before_mode(self):
        from multitask.control import MotionSafetyError
        from multitask.piper_backend import (
            PiperSdkAdapter,
            issue_hardware_authorization,
        )
        sdk = SdkStub()
        adapter = PiperSdkAdapter(
            sdk_factory=lambda *args, **kwargs: sdk,
            authorization=issue_hardware_authorization(
                armed=True, allow_hardware=True))
        adapter.connect()
        cases = (
            ("INVALID_JOINT_COUNT", [0.0] * 5, 10),
            ("NON_FINITE_JOINT", [math.nan, 0, 0, 0, 0, 0], 10),
            ("JOINT_LIMIT", [math.radians(151), 0, 0, 0, 0, 0], 10),
            ("SDK_SPEED_LIMIT", [0.0] * 6, 0),
            ("SDK_SPEED_LIMIT", [0.0] * 6, 21),
            ("SDK_SPEED_LIMIT", [0.0] * 6, 100),
        )
        for expected_code, joints, speed in cases:
            with self.subTest(code=expected_code, speed=speed):
                before = list(sdk.events)
                with self.assertRaises(MotionSafetyError) as caught:
                    adapter.command_joint_positions(joints, speed)
                self.assertEqual(caught.exception.code, expected_code)
                self.assertEqual(caught.exception.to_dict()["code"], expected_code)
                self.assertEqual(sdk.events, before)

    def test_sdk_adapter_direct_gripper_bypass_keeps_hard_limit(self):
        from multitask.control import MotionSafetyError
        from multitask.piper_backend import (
            PiperSdkAdapter,
            issue_hardware_authorization,
        )
        sdk = SdkStub()
        adapter = PiperSdkAdapter(
            sdk_factory=lambda *args, **kwargs: sdk,
            authorization=issue_hardware_authorization(
                armed=True, allow_hardware=True))
        adapter.connect()
        for opening in (-0.001, 0.071, math.inf):
            with self.subTest(opening=opening):
                before = list(sdk.events)
                with self.assertRaises(MotionSafetyError) as caught:
                    adapter.command_gripper(opening, 900)
                self.assertEqual(caught.exception.code, "GRIPPER_RANGE")
                self.assertEqual(sdk.events, before)

    def test_sdk_connect_partial_success_disconnects_before_reraising(self):
        from multitask.piper_backend import (
            PiperSdkAdapter,
            issue_hardware_authorization,
        )
        sdk = ConnectRaisesSdkStub()
        adapter = PiperSdkAdapter(
            sdk_factory=lambda *args, **kwargs: sdk,
            authorization=issue_hardware_authorization(
                armed=True, allow_hardware=True))
        with self.assertRaisesRegex(RuntimeError, "connect boom"):
            adapter.connect()
        self.assertEqual(sdk.events,
                         [("ConnectPort",), ("DisconnectPort",)])
        self.assertFalse(adapter._connected)

    def test_sdk_missing_required_callable_disconnects_constructed_object(self):
        from multitask.piper_backend import (
            PiperDependencyError,
            PiperSdkAdapter,
            issue_hardware_authorization,
        )
        sdk = SdkStub()
        sdk.JointCtrl = None
        adapter = PiperSdkAdapter(
            sdk_factory=lambda *args, **kwargs: sdk,
            authorization=issue_hardware_authorization(
                armed=True, allow_hardware=True))
        with self.assertRaisesRegex(PiperDependencyError, "JointCtrl"):
            adapter.connect()
        self.assertEqual(sdk.events, [("DisconnectPort",)])

    def test_sdk_connect_preserves_primary_and_attaches_disconnect_error(self):
        from multitask.piper_backend import (
            PiperSdkAdapter,
            issue_hardware_authorization,
        )
        sdk = ConnectAndDisconnectRaiseSdkStub()
        adapter = PiperSdkAdapter(
            sdk_factory=lambda *args, **kwargs: sdk,
            authorization=issue_hardware_authorization(
                armed=True, allow_hardware=True))
        with self.assertRaisesRegex(RuntimeError, "connect boom") as caught:
            adapter.connect()
        self.assertEqual(sdk.events,
                         [("ConnectPort",), ("DisconnectPort",)])
        self.assertIn("disconnect boom", " ".join(
            getattr(caught.exception, "cleanup_errors", ())))

    def test_backend_connect_failure_always_closes_adapter(self):
        from multitask.piper_backend import (
            PiperHardwareBackend,
            issue_hardware_authorization,
        )

        class FailingAdapter:
            def __init__(self):
                self.events = []

            def connect(self):
                self.events.append("connect")
                raise RuntimeError("adapter connect failed")

            def close(self):
                self.events.append("close")

        adapter = FailingAdapter()
        backend = PiperHardwareBackend(
            adapter,
            authorization=issue_hardware_authorization(
                armed=True, allow_hardware=True))
        error = self.assert_error_code("CONNECT_FAILED", backend.connect)
        self.assertIn("adapter connect failed", str(error))
        self.assertEqual(adapter.events, ["connect", "close"])

    def test_backend_connect_preserves_cleanup_failure_as_details(self):
        from multitask.piper_backend import (
            PiperHardwareBackend,
            issue_hardware_authorization,
        )

        class FailingAdapter:
            def connect(self):
                raise RuntimeError("adapter connect failed")

            def close(self):
                raise RuntimeError("adapter close failed")

        backend = PiperHardwareBackend(
            FailingAdapter(),
            authorization=issue_hardware_authorization(
                armed=True, allow_hardware=True))
        error = self.assert_error_code("CONNECT_FAILED", backend.connect)
        self.assertIn("adapter close failed", error.details["cleanup_error"])

    def test_sdk_default_models_missing_per_pair_freshness_fail_closed(self):
        from multitask.piper_backend import (
            PiperHardwareBackend,
            PiperSdkAdapter,
            issue_hardware_authorization,
        )
        sdk = SdkStub()
        token = issue_hardware_authorization(armed=True, allow_hardware=True)
        adapter = PiperSdkAdapter(
            sdk_factory=lambda *args, **kwargs: sdk,
            authorization=token, clock=lambda: 10.0)
        self.assertFalse(adapter.capabilities.per_pair_freshness)
        backend = PiperHardwareBackend(
            adapter, authorization=token, clock=lambda: 10.0,
            sleeper=lambda seconds: None)
        backend.connect()
        error = self.assert_error_code("JOINT_FRESHNESS_UNPROVEN", backend.enable)
        self.assertIn("per-pair", str(error))
        self.assertIn(("EmergencyStop", 0x01), sdk.events)
        self.assertEqual(sdk.events[-1], ("DisconnectPort",))

    def test_sdk_aggregate_refresh_cannot_mask_one_stale_joint_pair(self):
        from multitask.piper_backend import (
            PiperSdkAdapter,
            issue_hardware_authorization,
        )
        sdk = SdkStub()
        adapter = PiperSdkAdapter(
            sdk_factory=lambda *args, **kwargs: sdk,
            authorization=issue_hardware_authorization(
                armed=True, allow_hardware=True),
            pair_timestamp_reader=lambda message: message.pair_timestamps,
            clock=lambda: 10.0)
        adapter.connect()
        first = adapter.read_state()
        sdk.joint_timestamp += 1.0
        sdk.status_timestamp += 1.0
        sdk.pair_timestamps = (2.0, 1.0, 2.0)
        second = adapter.read_state()
        self.assertTrue(first.per_pair_freshness)
        self.assertFalse(second.per_pair_freshness)
        self.assertEqual(second.timestamp_s, first.timestamp_s)

    def test_sdk_nonzero_status_and_enable_flags_are_preserved(self):
        from multitask.piper_backend import (
            PiperSdkAdapter,
            issue_hardware_authorization,
        )
        sdk = SdkStub()
        sdk.status_code = 1
        sdk.enabled[-1] = False
        adapter = PiperSdkAdapter(
            sdk_factory=lambda *args, **kwargs: sdk,
            authorization=issue_hardware_authorization(
                armed=True, allow_hardware=True),
            pair_timestamp_reader=lambda message: message.pair_timestamps)
        adapter.connect()
        state = adapter.read_state()
        self.assertEqual(state.status_code, "0x01")
        self.assertTrue(state.faulted)
        self.assertTrue(state.stopped)
        self.assertFalse(state.enabled)

    def test_sdk_feedback_requires_joint_and_status_rates_and_timestamps(self):
        from multitask.piper_backend import (
            PiperDependencyError,
            PiperSdkAdapter,
            issue_hardware_authorization,
        )
        for field in ("joint_hz", "status_hz"):
            with self.subTest(field=field):
                sdk = SdkStub()
                setattr(sdk, field, 0.0)
                adapter = PiperSdkAdapter(
                    sdk_factory=lambda *args, _sdk=sdk, **kwargs: _sdk,
                    authorization=issue_hardware_authorization(
                        armed=True, allow_hardware=True))
                adapter.connect()
                with self.assertRaises(PiperDependencyError):
                    adapter.read_state()


if __name__ == "__main__":
    unittest.main()
