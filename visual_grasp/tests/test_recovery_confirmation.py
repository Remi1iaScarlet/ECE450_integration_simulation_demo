from unittest.mock import patch

from multitask.executor import TaskExecutor


class FakeWorld:
    def body_z(self, name):
        return 0.15

    def body_pos(self, name):
        return None


CFG = {
    "detector": {"scan_poses": [], "confidence_threshold": 0.3,
                 "backend": "yolo", "radius_correction": 1.0},
    "objects": {"graspable": ["cup"], "containers": [], "body_of": {}},
    "scene": {"table_top_z": 0.12},
    "grasp_planner": {"operating_domain": {"x": [0.3, 0.46], "y": [-0.22, 0.02]}},
    "grasp": {"strategy": "multi_candidate_v1_1"},
}


def test_two_recovery_misses_become_target_lost_not_out_of_domain():
    executor = TaskExecutor(FakeWorld(), CFG, verbose=False)
    ctx = {"source_label": "cup", "policy": "nearest", "observation_version": 1,
           "awaiting_reobservation": True}
    with patch("multitask.executor.object_registry.scan", side_effect=[[], []]) as scan:
        result = executor._st_detect_source(ctx)
    assert scan.call_count == 2
    assert not result["success"]
    assert result["data"]["reason"] == "TARGET_LOST_AFTER_RECOVERY"
    assert executor.log[-1]["reason"] == "TARGET_NOT_OBSERVED_AFTER_RECOVERY"


def test_confirmation_scan_can_recover_target_and_clear_waiting_state():
    executor = TaskExecutor(FakeWorld(), CFG, verbose=False)
    ctx = {"source_label": "cup", "policy": "nearest", "observation_version": 1,
           "awaiting_reobservation": True}
    obj = {"id": "cup:1", "label": "cup", "graspable": True,
           "center_base": [0.38, -0.11, 0.16], "confidence": 0.9, "n_roi": 20}
    with patch("multitask.executor.object_registry.scan", side_effect=[[], [obj]]):
        result = executor._st_detect_source(ctx)
    assert result["success"]
    assert "awaiting_reobservation" not in ctx
    assert executor.log[-1]["state"] == "CONFIRM_SOURCE"
    assert executor.log[-1]["success"]
