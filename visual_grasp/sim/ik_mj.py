"""MuJoCo Jacobian-based numerical IK for the Piper arm (Phase 5).

Drives a target site (the gripper TCP) to a desired world pose using damped
least-squares on the site Jacobian, restricted to the 6 arm DOFs. Works directly
in MuJoCo frames, so it avoids the DH/TCP frame mismatch and the analytic Pieper
solver's reachability failures (see DEVLOG D4/D7).
"""
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

ARM_DOFS = list(range(6))  # joint1..joint6 are the first 6 DOFs


def solve_ik(model, data, site_id, target_pos, target_mat=None, target_xaxis=None,
             target_horizontal_axis=None,
             iters=500, pos_tol=1e-3, rot_tol=1e-2, lam=0.1, step=0.5):
    """Returns (q_arm, converged). Mutates data.qpos[0:6] toward the solution."""
    lo = model.jnt_range[ARM_DOFS, 0]
    hi = model.jnt_range[ARM_DOFS, 1]
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    use_rot = (
        target_mat is not None
        or target_xaxis is not None
        or target_horizontal_axis is not None
    )
    converged = False
    for _ in range(iters):
        mujoco.mj_forward(model, data)
        cur_pos = data.site_xpos[site_id]
        perr = target_pos - cur_pos
        if target_mat is not None:
            cur_mat = data.site_xmat[site_id].reshape(3, 3)
            rerr = Rotation.from_matrix(target_mat @ cur_mat.T).as_rotvec()
            err = np.concatenate([perr, rerr])
        elif target_xaxis is not None:
            cur_mat = data.site_xmat[site_id].reshape(3, 3)
            cur_x = cur_mat[:, 0]
            target_x = target_xaxis / np.linalg.norm(target_xaxis)
            rerr = np.cross(cur_x, target_x)
            err = np.concatenate([perr, rerr])
        elif target_horizontal_axis is not None:
            cur_mat = data.site_xmat[site_id].reshape(3, 3)
            cur_axis = cur_mat[:, target_horizontal_axis]
            up = np.array([0.0, 0.0, 1.0])
            vertical = float(cur_axis @ up)
            # Rotate around the direction that reduces the selected axis' vertical
            # component. This keeps the axis horizontal without fixing yaw.
            rerr = -vertical * np.cross(cur_axis, up)
            err = np.concatenate([perr, rerr])
        else:
            err = perr
        if np.linalg.norm(perr) < pos_tol and (not use_rot or np.linalg.norm(rerr) < rot_tol):
            converged = True
            break
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        J = np.vstack([jacp, jacr])[:, ARM_DOFS] if use_rot else jacp[:, ARM_DOFS]
        # damped least squares
        dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(J.shape[0]), err)
        q = data.qpos[ARM_DOFS] + step * dq
        data.qpos[ARM_DOFS] = np.clip(q, lo, hi)
    mujoco.mj_forward(model, data)
    return data.qpos[ARM_DOFS].copy(), converged


def top_down(yaw=0.0):
    """Grasp orientation: tool +z points straight down; yaw about vertical."""
    Rtd = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
    return Rotation.from_euler("z", yaw).as_matrix() @ Rtd


def pitched(pitch_deg, yaw=0.0):
    """Tool +z tilted from straight-down by pitch_deg toward +x (forward)."""
    Rp = Rotation.from_euler("y", np.deg2rad(pitch_deg)).as_matrix()
    return Rp @ top_down(yaw)
