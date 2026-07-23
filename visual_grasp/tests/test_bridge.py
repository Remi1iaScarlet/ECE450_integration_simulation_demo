import json
import subprocess
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import numpy as np

from multitask.bridge import BridgeCommandError, SCHEMA_VERSION, normalize_command, run_bridge


class OfflineClock:
    def __init__(self):
        self.now = 10.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += float(seconds)


class OfflineAdapter:
    def __init__(self, clock, events):
        self.clock = clock
        self.events = events
        self.current = [0.0] * 6
        self.connected = False
        self.enabled = False

    def connect(self):
        self.connected = True
        self.events.append("connect")

    def enable(self):
        self.enabled = True
        self.events.append("enable")

    def read_state(self):
        from multitask.piper_backend import RobotState
        return RobotState(
            tuple(self.current), self.clock.monotonic(),
            self.connected, self.enabled,
            per_pair_freshness=True,
            freshness_evidence=("bridge_offline_fake",))

    def command_joint_positions(self, joints, speed_percent):
        self.current = [float(value) for value in joints]
        self.events.append(("joint", tuple(self.current), speed_percent))

    def command_gripper(self, opening_m, effort):
        self.events.append(("gripper", float(opening_m), int(effort)))

    def stop(self):
        self.enabled = False
        self.events.append("stop")

    def close(self):
        self.connected = False
        self.events.append("adapter_close")


def pick_motion_plan():
    from multitask.motion_plan import JointWaypoint, MotionPlan, MotionSegment
    points = [np.array([value, 0, 0, 0, 0, 0], dtype=float)
              for value in (0.0, 0.05, 0.10, 0.15)]
    return MotionPlan(points[0], [
        MotionSegment("start_to_pre", "TRANSIT", [
            JointWaypoint("start", points[0]), JointWaypoint("pre", points[1])]),
        MotionSegment("pre_to_grasp", "FINAL_APPROACH", [
            JointWaypoint("pre", points[1]), JointWaypoint("grasp", points[2])]),
        MotionSegment("grasp_to_lift", "LIFT", [
            JointWaypoint("grasp", points[2]), JointWaypoint("lift", points[3])]),
    ])


class BridgeCommandTests(unittest.TestCase):
    def test_place_into_normalizes_target_alias(self):
        cmd = normalize_command({
            "task": "place_into",
            "source": "cup",
            "target": "bowl",
            "backend": "sim",
            "perception": "sim_gt",
            "speed": 0.3,
        })

        self.assertEqual(cmd["schema_version"], SCHEMA_VERSION)
        self.assertEqual(cmd["task"], "place_into")
        self.assertEqual(cmd["source_label"], "cup")
        self.assertEqual(cmd["container_label"], "bowl")
        self.assertEqual(cmd["backend"], "sim_mujoco")
        self.assertEqual(cmd["perception"], "sim_gt")
        self.assertEqual(cmd["speed"], 0.3)

    def test_clear_table_accepts_comma_separated_labels(self):
        cmd = normalize_command({
            "task": "clear_table",
            "labels": "cup,bottle",
            "target": "bowl",
        })

        self.assertEqual(cmd["labels"], ["cup", "bottle"])
        self.assertEqual(cmd["container_label"], "bowl")

    def test_missing_required_argument_is_rejected(self):
        with self.assertRaises(BridgeCommandError):
            normalize_command({"task": "place_at", "source": "cup"})

    def test_dry_run_does_not_need_sim_dependencies(self):
        result = run_bridge(
            json.dumps({"task": "pick", "source": "cup", "backend": "sim"}),
            dry_run=True,
        )

        self.assertTrue(result["success"])
        self.assertTrue(result["dry_run"])
        self.assertEqual(result["command"]["task"], "pick")
        self.assertEqual(result["dispatch"]["call"], "TaskExecutor.pick")

    def test_real_piper_rejects_sim_ground_truth_even_in_dry_run(self):
        with self.assertRaisesRegex(BridgeCommandError, "sim_gt"):
            run_bridge({
                "task": "pick", "source": "cup", "backend": "real_piper",
                "perception": "sim_gt",
            }, dry_run=True)

    def test_armed_must_be_a_real_boolean(self):
        with self.assertRaisesRegex(BridgeCommandError, "armed"):
            normalize_command({
                "task": "pick", "source": "cup", "backend": "real_piper",
                "armed": "true",
            })

    def test_real_piper_dry_run_never_constructs_injected_factories(self):
        def forbidden(*args, **kwargs):
            raise AssertionError("dry-run constructed a real dependency")

        result = run_bridge({
            "task": "pick", "source": "cup", "backend": "real_piper",
            "perception": "yolo", "armed": True,
        }, dry_run=True, backend_factory=forbidden,
           perception_factory=forbidden, executor_factory=forbidden,
           allow_hardware=True)
        self.assertTrue(result["dry_run"])
        self.assertEqual(
            result["dispatch"]["call"], "RealPiperTaskExecutor.pick")

    def test_real_piper_requires_command_and_api_hardware_gates(self):
        def forbidden(*args, **kwargs):
            raise AssertionError("gate failure constructed a real dependency")

        base = {
            "task": "pick", "source": "cup", "backend": "real_piper",
            "perception": "yolo",
        }
        with self.assertRaisesRegex(BridgeCommandError, "armed"):
            run_bridge(base, allow_hardware=True, backend_factory=forbidden,
                       perception_factory=forbidden, executor_factory=forbidden)
        with self.assertRaisesRegex(BridgeCommandError, "allow_hardware"):
            run_bridge({**base, "armed": True}, allow_hardware=False,
                       backend_factory=forbidden, perception_factory=forbidden,
                       executor_factory=forbidden)

    def test_real_piper_dispatches_only_through_injected_dependencies(self):
        events = []

        class Backend:
            def connect(self):
                events.append("connect")

            def enable(self):
                events.append("enable")

            def close(self):
                events.append("close")

        backend = Backend()
        perception = object()

        class Executor:
            def pick(self, source, policy):
                events.append(("pick", source, policy, perception))
                return {"success": True, "message": "offline injected dispatch"}

        result = run_bridge({
            "task": "pick", "source": "cup", "backend": "real_piper",
            "perception": "yolo", "armed": True,
        }, allow_hardware=True,
           backend_factory=lambda command, authorization: backend,
           perception_factory=lambda command: perception,
           executor_factory=lambda backend, perception, command: Executor())
        self.assertTrue(result["success"])
        self.assertEqual(events[:2], ["connect", "enable"])
        self.assertEqual(events[-1], "close")
        self.assertEqual(events[2][:3], ("pick", "cup", "nearest"))

    def test_default_real_executor_preflight_rejects_before_connect(self):
        from multitask.real_piper_executor import RealPiperExecutorError
        events = []

        class Backend:
            def connect(self): events.append("connect")
            def enable(self): events.append("enable")
            def close(self): events.append("close")

        class Planner:
            def build_pick_motion_plan(self, *args, **kwargs):
                raise AssertionError("planning must not run during preflight")

        with self.assertRaisesRegex(RealPiperExecutorError, "locate"):
            run_bridge({
                "task": "pick", "source": "cup", "backend": "real_piper",
                "perception": "yolo", "armed": True,
            }, allow_hardware=True,
               backend_factory=lambda command, authorization: Backend(),
               perception_factory=lambda command: object(),
               planner_factory=lambda command: Planner())
        self.assertEqual(events, ["close"])

    def test_provider_dependency_preflight_fails_before_connect(self):
        events = []

        class Backend:
            def connect(self): events.append("connect")
            def enable(self): events.append("enable")
            def close(self): events.append("close")

        class Provider:
            def preflight(self):
                raise ModuleNotFoundError("missing camera dependency")

            def locate(self, label):
                raise AssertionError("perception must not run during preflight")

        class Planner:
            def build_pick_motion_plan(self, *args, **kwargs):
                raise AssertionError("planning must not run during preflight")

        with self.assertRaisesRegex(ModuleNotFoundError,
                                    "missing camera dependency"):
            run_bridge({
                "task": "pick", "source": "cup", "backend": "real_piper",
                "perception": "yolo", "armed": True,
            }, allow_hardware=True,
               backend_factory=lambda command, authorization: Backend(),
               perception_factory=lambda command: Provider(),
               planner_factory=lambda command: Planner())
        self.assertEqual(events, ["close"])

    def test_real_piper_missing_provider_configuration_is_explicit_and_non_connecting(self):
        events = []

        class Backend:
            def connect(self):
                raise AssertionError("must not connect without an executor factory")

            def close(self):
                events.append("close")

        with self.assertRaisesRegex(BridgeCommandError, "provider"):
            run_bridge({
                "task": "pick", "source": "cup", "backend": "real_piper",
                "perception": "yolo", "armed": True,
            }, allow_hardware=True,
               backend_factory=lambda command, authorization: Backend())
        self.assertEqual(events, ["close"])

    def test_default_real_executor_rejects_unimplemented_task_before_construction(self):
        def forbidden(*args, **kwargs):
            raise AssertionError("unsupported default task constructed a dependency")

        with self.assertRaisesRegex(BridgeCommandError, "only pick"):
            run_bridge({
                "task": "place_at", "source": "cup", "xyz": [0.3, 0, 0.2],
                "backend": "real_piper", "perception": "yolo", "armed": True,
            }, allow_hardware=True, backend_factory=forbidden,
               perception_factory=forbidden, planner_factory=forbidden)

    def test_real_piper_executor_failure_still_closes_backend(self):
        events = []

        class Backend:
            def connect(self): events.append("connect")
            def enable(self): events.append("enable")
            def close(self): events.append("close")

        class Executor:
            def pick(self, source, policy):
                raise RuntimeError("executor boom")

        with self.assertRaisesRegex(RuntimeError, "executor boom"):
            run_bridge({
                "task": "pick", "source": "cup", "backend": "real_piper",
                "perception": "yolo", "armed": True,
            }, allow_hardware=True,
               backend_factory=lambda command, authorization: Backend(),
               perception_factory=lambda command: object(),
               executor_factory=lambda backend, perception, command: Executor())
        self.assertEqual(events, ["connect", "enable", "close"])

    def test_authorization_capability_is_issued_before_backend_factory(self):
        captured = []

        class Backend:
            def connect(self): captured.append("connect")
            def enable(self): captured.append("enable")
            def close(self): captured.append("close")

        class Executor:
            def pick(self, source, policy):
                return {"success": True, "message": "ok"}

        def backend_factory(command, authorization):
            from multitask.piper_backend import HardwareAuthorization
            self.assertIsInstance(authorization, HardwareAuthorization)
            captured.append("factory_with_capability")
            return Backend()

        run_bridge({
            "task": "pick", "source": "cup", "backend": "real_piper",
            "perception": "yolo", "armed": True,
        }, allow_hardware=True, backend_factory=backend_factory,
           perception_factory=lambda command: object(),
           executor_factory=lambda backend, perception, command: Executor())
        self.assertEqual(captured[0], "factory_with_capability")

    def test_perception_factory_failure_closes_constructed_backend(self):
        events = []

        class Backend:
            def close(self): events.append("backend_close")

        def fail_perception(command):
            raise RuntimeError("provider factory boom")

        with self.assertRaisesRegex(RuntimeError, "provider factory boom"):
            run_bridge({
                "task": "pick", "source": "cup", "backend": "real_piper",
                "perception": "yolo", "armed": True,
            }, allow_hardware=True,
               backend_factory=lambda command, authorization: Backend(),
               perception_factory=fail_perception,
               executor_factory=lambda *args: object())
        self.assertEqual(events, ["backend_close"])

    def test_executor_factory_failure_closes_perception_then_backend(self):
        events = []

        class Backend:
            def close(self): events.append("backend_close")

        class Perception:
            def close(self): events.append("perception_close")

        def fail_executor(*args):
            raise RuntimeError("executor factory boom")

        with self.assertRaisesRegex(RuntimeError, "executor factory boom"):
            run_bridge({
                "task": "pick", "source": "cup", "backend": "real_piper",
                "perception": "yolo", "armed": True,
            }, allow_hardware=True,
               backend_factory=lambda command, authorization: Backend(),
               perception_factory=lambda command: Perception(),
               executor_factory=fail_executor)
        self.assertEqual(events, ["perception_close", "backend_close"])

    def test_default_real_executor_runs_actual_motion_plan_through_fake_backend(self):
        from multitask.piper_backend import PiperHardwareBackend
        events = []
        clock = OfflineClock()

        def backend_factory(command, authorization):
            return PiperHardwareBackend(
                OfflineAdapter(clock, events), authorization=authorization,
                clock=clock.monotonic, sleeper=clock.sleep)

        class Provider:
            def locate(self, label):
                events.append(("locate", label))
                return {"label": label, "center_base": [0.3, 0.0, 0.15]}

        class Planner:
            def build_pick_motion_plan(self, observation, source_label, policy):
                events.append(("plan", observation["label"], source_label, policy))
                return pick_motion_plan()

        result = run_bridge({
            "task": "pick", "source": "cup", "backend": "real_piper",
            "perception": "yolo", "armed": True,
        }, allow_hardware=True, backend_factory=backend_factory,
           perception_factory=lambda command: Provider(),
           planner_factory=lambda command: Planner())
        self.assertTrue(result["success"])
        self.assertEqual(len([event for event in events
                              if isinstance(event, tuple) and
                              event[0] == "gripper"]), 1)
        gripper_index = next(index for index, event in enumerate(events)
                             if isinstance(event, tuple) and event[0] == "gripper")
        lift_index = next(index for index, event in enumerate(events)
                          if isinstance(event, tuple) and event[0] == "joint" and
                          event[1][0] > 0.10)
        self.assertLess(gripper_index, lift_index)
        self.assertEqual(events[-2:], ["stop", "adapter_close"])

    def test_factory_import_paths_are_preserved_by_command_contract(self):
        command = normalize_command({
            "task": "pick", "source": "cup", "backend": "real_piper",
            "perception": "yolo", "armed": True,
            "provider_factory": "package.providers:create_provider",
            "planner_factory": "package.planners:create_planner",
        })
        self.assertEqual(command["provider_factory"],
                         "package.providers:create_provider")
        self.assertEqual(command["planner_factory"],
                         "package.planners:create_planner")

    def test_configured_factory_supports_callable_and_nested_attribute(self):
        from multitask.bridge import _configured_factory
        module = ModuleType("test_real_factories")
        top_level = lambda command: ("provider", command)
        nested = lambda command: ("planner", command)
        module.create_provider = top_level
        module.factories = SimpleNamespace(create_planner=nested)
        with patch.dict(sys.modules, {module.__name__: module}):
            self.assertIs(_configured_factory(
                "test_real_factories:create_provider", "provider"), top_level)
            self.assertIs(_configured_factory(
                "test_real_factories:factories.create_planner", "planner"),
                nested)

    def test_configured_factory_rejects_noncallable_and_bad_paths(self):
        from multitask.bridge import _configured_factory
        module = ModuleType("test_bad_real_factories")
        module.not_callable = 7
        with patch.dict(sys.modules, {module.__name__: module}):
            with self.assertRaisesRegex(BridgeCommandError, "not callable"):
                _configured_factory(
                    "test_bad_real_factories:not_callable", "provider")
            with self.assertRaisesRegex(BridgeCommandError, "cannot import"):
                _configured_factory(
                    "test_bad_real_factories:missing.factory", "planner")
        with self.assertRaisesRegex(BridgeCommandError, "module:callable"):
            _configured_factory("test_bad_real_factories", "provider")
        with self.assertRaisesRegex(BridgeCommandError, "cannot import"):
            _configured_factory("definitely_missing_real_module:create", "planner")

    def test_command_factory_paths_dispatch_real_pick_through_fake_backend(self):
        from multitask.motion_plan import MotionPlan
        events = []
        module = ModuleType("test_command_real_factories")

        class Provider:
            def locate(self, label):
                events.append(("locate", label))
                return {"label": label}

        class Planner:
            def build_pick_motion_plan(self, observation, source_label, policy):
                events.append(("plan", observation["label"], policy))
                return pick_motion_plan()

        module.create_provider = lambda command: Provider()
        module.nested = SimpleNamespace(create_planner=lambda command: Planner())

        class Backend:
            def connect(self): events.append("connect")
            def enable(self): events.append("enable")

            def execute_grasp_plan(self, plan):
                assert isinstance(plan, MotionPlan)
                events.append("execute")
                return {"segments": 3}

            def close(self): events.append("close")

        with patch.dict(sys.modules, {module.__name__: module}):
            result = run_bridge({
                "task": "pick", "source": "cup", "backend": "real_piper",
                "perception": "yolo", "armed": True,
                "provider_factory": "test_command_real_factories:create_provider",
                "planner_factory": (
                    "test_command_real_factories:nested.create_planner"),
            }, allow_hardware=True,
               backend_factory=lambda command, authorization: Backend())
        self.assertTrue(result["success"])
        self.assertEqual(events[:2], ["connect", "enable"])
        self.assertIn("execute", events)
        self.assertEqual(events[-1], "close")

    def test_neutral_real_imports_and_preflight_work_when_sim_stacks_blocked(self):
        code = r'''
import builtins
real_import = builtins.__import__
blocked = {"mujoco", "ultralytics", "rospy"}
def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
    if name.split(".", 1)[0] in blocked:
        raise ModuleNotFoundError("blocked dependency: " + name)
    return real_import(name, globals, locals, fromlist, level)
builtins.__import__ = guarded_import
from multitask.motion_plan import JointWaypoint, MotionPlan, MotionSegment
from multitask.real_piper_executor import RealPiperTaskExecutor
from multitask.bridge import run_bridge
events = []
class Backend:
    def connect(self): events.append("connect")
    def enable(self): events.append("enable")
    def execute_grasp_plan(self, plan):
        assert isinstance(plan, MotionPlan)
        events.append("execute")
        return {"ok": True}
    def close(self): events.append("close")
class Provider:
    def locate(self, label): return {"label": label}
class Planner:
    def build_pick_motion_plan(self, observation, source_label, policy):
        q0 = [0.0] * 6
        q1 = [0.1, 0, 0, 0, 0, 0]
        return MotionPlan(q0, [MotionSegment("move", "LIFT", [
            JointWaypoint("start", q0), JointWaypoint("end", q1)])])
executor = RealPiperTaskExecutor(Backend(), Provider(), Planner())
executor.preflight()
result = run_bridge(
    {"task": "pick", "source": "cup", "backend": "real_piper",
     "perception": "yolo", "armed": True},
    allow_hardware=True,
    backend_factory=lambda command, authorization: Backend(),
    perception_factory=lambda command: Provider(),
    planner_factory=lambda command: Planner())
assert result["success"]
assert events == ["connect", "enable", "execute", "close"]
assert not any(name.split(".", 1)[0] in blocked for name in sys.modules)
'''
        result = subprocess.run(
            [sys.executable, "-c", "import sys\n" + code],
            capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_sim_bridge_applies_active_model_scene_ik_and_overrides_without_viewer(self):
        captured = {}
        cfg = {
            "model": {
                "active": "real",
                "real": {
                    "scene": "models/piper_scene.xml",
                    "analytic_ik": True,
                    "scan_poses": [[0.1] * 6],
                    "detector": {"confidence_threshold": 0.42},
                    "scene": "models/piper_scene.xml",
                },
            },
            "detector": {"backend": "yolo", "confidence_threshold": 0.2},
            "objects": {"graspable": ["cup"], "containers": ["bowl"]},
            "grasp": {}, "place": {}, "scene": {},
        }

        def world_factory(**kwargs):
            captured.update(kwargs)
            return object()

        class Executor:
            def pick(self, source, policy):
                return {"success": True, "message": "sim fake"}

        result = run_bridge({
            "task": "pick", "source": "cup", "backend": "sim",
            "perception": "yolo",
        }, no_evidence=True,
           config_loader=lambda: cfg,
           sim_world_factory=world_factory,
           sim_executor_factory=lambda world, resolved_cfg: Executor())
        self.assertTrue(result["success"])
        self.assertTrue(captured["use_analytic_ik"])
        self.assertTrue(str(captured["scene"]).endswith("models/piper_scene.xml"))
        self.assertEqual(cfg["detector"]["scan_poses"], [[0.1] * 6])
        self.assertEqual(captured["annotate_conf"], 0.42)


if __name__ == "__main__":
    unittest.main()
