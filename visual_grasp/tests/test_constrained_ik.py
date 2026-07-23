import numpy as np

from sim.constrained_ik import IKConstraints, IKResult, IKStage


def test_constrained_ik_public_result_contract():
    result = IKResult(True, np.zeros(6), 0.001, 2.0, 3.0, "RELAXED_ORIENTATION",
                      "bounded_numeric_seed_0", 12, 4.5, "CONVERGED_BOUNDED")
    assert result.success
    assert result.tcp_error_m <= IKConstraints().position_tolerance_m
    assert result.iterations == 12
    assert result.elapsed_ms == 4.5
    assert result.termination_reason == "CONVERGED_BOUNDED"


def test_constrained_ik_stage_order_values_are_stable():
    assert [IKStage.GRASP.value, IKStage.PRE_GRASP.value, IKStage.LIFT.value] == [
        "GRASP", "PRE_GRASP", "LIFT"]
