"""Phase 1 — DH-FK vs MuJoCo alignment check.

Goal: verify that piper_arm.py's DH forward kinematics agrees with the Menagerie
MuJoCo model, so that piper_arm.py's IK output is trustworthy in simulation.

Strategy:
  - MuJoCo is ground truth. base_link sits at the world origin, so world == base.
  - For a joint vector q (real-robot convention), set MuJoCo qpos[0:6] = signs * q,
    mj_forward, read link6 body world pose.
  - Compare against piper_arm.forward_kinematics(q) (which applies theta_offset).
  - Brute-force the per-joint sign pattern in {+1,-1}^6 over several test configs to
    find the mapping that minimises mean link6 position error.
  - Report whether the DH->MJCF rotation offset is constant (a fixed frame-convention
    difference) rather than configuration-dependent (a bug).

Headless: requires MUJOCO_GL=egl. Run from anywhere:
    MUJOCO_GL=egl python3 sim/phase1_fk_check.py
"""
import itertools
import os
import pathlib
import sys

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from piper_arm import PiperArm  # noqa: E402

SCENE = ROOT / "sim" / "models" / "piper" / "scene.xml"
np.set_printoptions(precision=4, suppress=True)

# Test configs in real-robot convention (rad). Kept inside the tighter of the two
# joint ranges so signs don't push us far outside model limits.
TEST_Q = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.3, 0.6, -0.5, 0.0, 0.0, 0.0],
    [-0.4, 1.0, -0.8, 0.2, 0.3, 0.0],
    [0.5, 1.2, -1.0, -0.3, -0.2, 0.4],
    [-0.6, 0.4, -0.3, 0.4, 0.1, -0.5],
    [0.2, 1.5, -1.3, 0.5, 0.4, 0.6],
]


def mj_link6_pose(model, data, q6, signs):
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i in range(6):
        data.qpos[i] = signs[i] * q6[i]
    mujoco.mj_forward(model, data)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    return data.xpos[bid].copy(), data.xmat[bid].reshape(3, 3).copy()


def mean_pos_error(model, data, arm, signs):
    errs = []
    for q in TEST_Q:
        p_mj, _ = mj_link6_pose(model, data, q, signs)
        p_dh = arm.forward_kinematics(q)[0:3, 3]
        errs.append(np.linalg.norm(p_mj - p_dh))
    return float(np.mean(errs))


def main():
    arm = PiperArm()
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    # 1) Brute-force sign pattern.
    print("=== brute-force per-joint sign search (mean link6 pos error) ===")
    results = []
    for signs in itertools.product([1, -1], repeat=6):
        results.append((mean_pos_error(model, data, arm, signs), signs))
    results.sort(key=lambda r: r[0])
    for err, signs in results[:5]:
        print(f"  signs={signs}  mean_err={err*1000:.2f} mm")
    best_err, best_signs = results[0]
    print(f"\nBEST signs = {best_signs}  mean_err = {best_err*1000:.3f} mm")

    # 2) Per-config detail with best signs.
    print("\n=== per-config detail (best signs) ===")
    print(f"{'q (deg)':<42} {'p_mj':<26} {'p_dh':<26} err(mm)")
    rot_offsets = []
    for q in TEST_Q:
        p_mj, R_mj = mj_link6_pose(model, data, q, best_signs)
        T_dh = arm.forward_kinematics(q)
        p_dh, R_dh = T_dh[0:3, 3], T_dh[0:3, 0:3]
        err = np.linalg.norm(p_mj - p_dh) * 1000
        rot_offsets.append(R_mj.T @ R_dh)  # constant if only a frame-convention diff
        qdeg = np.array(q) * 180 / np.pi
        print(f"{np.array2string(qdeg, precision=1):<42} "
              f"{np.array2string(p_mj):<26} {np.array2string(p_dh):<26} {err:6.2f}")

    # 3) Is the DH->MJCF rotation offset constant?
    print("\n=== rotation offset R_mj^T @ R_dh (constant => fixed frame convention) ===")
    ref = rot_offsets[0]
    max_dev = max(np.linalg.norm(R - ref) for R in rot_offsets)
    print("offset @ q0:\n", ref)
    print(f"max deviation of offset across configs (Frobenius): {max_dev:.4f}")

    verdict = "POSITION ALIGNED" if best_err < 0.005 else "MISMATCH - investigate"
    print(f"\nVERDICT: {verdict} (best mean pos err {best_err*1000:.3f} mm)")


if __name__ == "__main__":
    main()
