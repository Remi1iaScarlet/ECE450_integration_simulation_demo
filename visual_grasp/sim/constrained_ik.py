"""Budgeted bounded-orientation IK adapter.

This module is additive: the legacy ``solve_ik`` entry points remain unchanged.
"""
from dataclasses import dataclass, field
from enum import Enum
import time

import mujoco
import numpy as np


class IKStage(str, Enum):
    GRASP = "GRASP"
    PRE_GRASP = "PRE_GRASP"
    LIFT = "LIFT"


class ConstraintLevel(str, Enum):
    EXACT = "EXACT"
    RELAXED_ORIENTATION = "RELAXED_ORIENTATION"


@dataclass(frozen=True)
class IKConstraints:
    position_tolerance_m: float = 0.002
    max_approach_error_deg: float = 20.0
    max_closing_axis_vertical_deg: float = 10.0
    wrist_equivalent_deg: tuple = (0.0, 180.0)


@dataclass
class IKRequest:
    model: object
    data: object
    site_id: int
    target_position: np.ndarray
    nominal_approach: np.ndarray
    nominal_closing_axis: np.ndarray
    constraints: IKConstraints = field(default_factory=IKConstraints)
    seeds: tuple = ()
    stage: IKStage = IKStage.GRASP
    max_iterations: int = 180
    time_budget_s: float = 0.20
    damping: float = 0.05
    step_size: float = 0.35
    exact_solver: object = None
    hard_lower: np.ndarray = None
    hard_upper: np.ndarray = None
    cache_namespace: str = "piper_real"


@dataclass
class IKResult:
    success: bool
    q: np.ndarray = None
    tcp_error_m: float = float("inf")
    approach_error_deg: float = float("inf")
    closing_axis_error_deg: float = float("inf")
    constraint_level: str = None
    solve_path: str = "none"
    iterations: int = 0
    elapsed_ms: float = 0.0
    termination_reason: str = "UNSOLVED"
    cache_hit: bool = False


_CACHE = {}


def clear_constrained_ik_cache():
    _CACHE.clear()


def _unit(v):
    v = np.asarray(v, float)
    return v / max(np.linalg.norm(v), 1e-12)


def _angle_deg(a, b, line=False):
    dot = float(np.clip(_unit(a) @ _unit(b), -1.0, 1.0))
    if line:
        dot = abs(dot)
    return float(np.rad2deg(np.arccos(dot)))


def _measure(req, data):
    mujoco.mj_forward(req.model, data)
    mat = data.site_xmat[req.site_id].reshape(3, 3)
    closing = mat[:, 1]
    return (float(np.linalg.norm(data.site_xpos[req.site_id] - req.target_position)),
            _angle_deg(mat[:, 2], req.nominal_approach),
            float(np.rad2deg(np.arcsin(np.clip(abs(closing[2]), 0.0, 1.0)))))


def _within_limits(q, lo, hi):
    return bool(np.all(q >= lo) and np.all(q <= hi))


def _cache_key(req, seeds, lo, hi):
    q = lambda a, s: tuple(np.rint(np.asarray(a, float) / s).astype(int))
    return (req.cache_namespace, req.stage.value, q(req.target_position, .002),
            q(_unit(req.nominal_approach), .02), q(_unit(req.nominal_closing_axis), .02),
            round(req.constraints.position_tolerance_m, 4),
            round(req.constraints.max_approach_error_deg, 2),
            round(req.constraints.max_closing_axis_vertical_deg, 2),
            tuple(q(s, .02) for s in seeds), q(lo, .01), q(hi, .01))


def solve_constrained_ik(req):
    """Solve once exactly, then once with native inequality constraints."""
    started = time.perf_counter()
    model = req.model
    lo = np.asarray(req.hard_lower if req.hard_lower is not None
                    else model.jnt_range[:6, 0], float)
    hi = np.asarray(req.hard_upper if req.hard_upper is not None
                    else model.jnt_range[:6, 1], float)
    seeds = [np.asarray(s, float) for s in req.seeds[:3]]
    if not seeds:
        seeds = [np.asarray(req.data.qpos[:6], float).copy()]
    seeds = [np.clip(s, lo, hi) for s in seeds]
    key = _cache_key(req, seeds, lo, hi)
    cached = _CACHE.get(key)
    if cached is not None:
        out = IKResult(**cached.__dict__)
        out.cache_hit = True
        out.elapsed_ms = (time.perf_counter() - started) * 1000.0
        return out

    if req.exact_solver is not None:
        q = req.exact_solver(req, seeds[0])
        if q is not None and _within_limits(np.asarray(q), lo, hi):
            check = mujoco.MjData(model); check.qpos[:] = req.data.qpos
            check.qpos[:6] = q
            pe, ae, ce = _measure(req, check)
            if pe <= req.constraints.position_tolerance_m and ae <= 0.5 and ce <= 0.5:
                out = IKResult(True, np.asarray(q), pe, ae, ce,
                               ConstraintLevel.EXACT.value, "analytic_exact", 0,
                               (time.perf_counter() - started) * 1000.0, "CONVERGED_EXACT")
                _CACHE[key] = out
                return out

    total_iters = 0
    jacp = np.zeros((3, model.nv)); jacr = np.zeros((3, model.nv))
    best = None
    for seed_index, seed in enumerate(seeds):
        data = mujoco.MjData(model); data.qpos[:] = req.data.qpos
        data.qvel[:] = 0; data.qpos[:6] = seed
        for _ in range(req.max_iterations):
            if time.perf_counter() - started >= req.time_budget_s:
                reason = "TIME_BUDGET_EXCEEDED"
                break
            total_iters += 1
            mujoco.mj_forward(model, data)
            mat = data.site_xmat[req.site_id].reshape(3, 3)
            perr = np.asarray(req.target_position) - data.site_xpos[req.site_id]
            ae = _angle_deg(mat[:, 2], req.nominal_approach)
            ce = float(np.rad2deg(np.arcsin(np.clip(abs(mat[2, 1]), 0.0, 1.0))))
            pe = float(np.linalg.norm(perr))
            score = pe / req.constraints.position_tolerance_m + \
                max(0., ae - req.constraints.max_approach_error_deg) + \
                max(0., ce - req.constraints.max_closing_axis_vertical_deg)
            if best is None or score < best[0]:
                best = (score, data.qpos[:6].copy(), pe, ae, ce)
            if (pe <= req.constraints.position_tolerance_m and
                    ae <= req.constraints.max_approach_error_deg and
                    ce <= req.constraints.max_closing_axis_vertical_deg):
                out = IKResult(True, data.qpos[:6].copy(), pe, ae, ce,
                               ConstraintLevel.RELAXED_ORIENTATION.value,
                               f"bounded_numeric_seed_{seed_index}", total_iters,
                               (time.perf_counter() - started) * 1000.0,
                               "CONVERGED_BOUNDED")
                _CACHE[key] = out
                return out
            # Only violated inequality residuals produce orientation forces.
            approach_cross = np.cross(mat[:, 2], _unit(req.nominal_approach))
            if ae <= req.constraints.max_approach_error_deg:
                approach_cross[:] = 0.0
            closing_residual = -mat[2, 1] * np.cross(mat[:, 1], [0., 0., 1.])
            if ce <= req.constraints.max_closing_axis_vertical_deg:
                closing_residual[:] = 0.0
            rerr = approach_cross + closing_residual
            mujoco.mj_jacSite(model, data, jacp, jacr, req.site_id)
            J = np.vstack([jacp, jacr])[:, :6]
            err = np.concatenate([perr, rerr])
            dq = J.T @ np.linalg.solve(J @ J.T + req.damping ** 2 * np.eye(6), err)
            data.qpos[:6] = np.clip(data.qpos[:6] + req.step_size * dq, lo, hi)
        else:
            reason = "MAX_ITERATIONS"
        if reason == "TIME_BUDGET_EXCEEDED":
            break
    pe, ae, ce = best[2:] if best else (float("inf"),) * 3
    out = IKResult(False, None, pe, ae, ce, None, "bounded_numeric", total_iters,
                   (time.perf_counter() - started) * 1000.0, reason)
    _CACHE[key] = out
    return out
