"""Phase 5 — simulated visual grasp closed loop (pick up the cup).

Pipeline:
  world_cam RGB -> YOLO (confirm cup detected) -> Route A ground-truth cup pose
  -> base_T_cup -> MuJoCo IK (TCP=finger-pad midpoint, horizontal gripper frame)
  for pre-grasp / grasp -> close gripper -> lift -> verify the cup rose.

Headless: requires MUJOCO_GL=egl.
    MUJOCO_GL=egl python3 sim/grasp_sim.py
"""
import os
import pathlib
import sys

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
from PIL import Image  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT))  # for piper_arm (analytic IK, shared with the real robot)
import ik_mj  # noqa: E402
import perception  # noqa: E402

_PIPER_ARM = None


def _piper_arm():
    """Lazily construct the shared DH analytic-IK model (piper_arm.PiperArm)."""
    global _PIPER_ARM
    if _PIPER_ARM is None:
        from piper_arm import PiperArm  # repo-root module (real-robot DH/IK)
        _PIPER_ARM = PiperArm()
    return _PIPER_ARM

SCENE = ROOT / "sim" / "models" / "piper" / "scene_grasp.xml"
OUT = ROOT / "sim" / "evidence"
GRIP_OPEN, GRIP_CLOSE = 0.035, -0.005  # ctrl[6]; slightly negative to clamp hard
np.set_printoptions(precision=4, suppress=True)


def horizontal_grasp_frame(target_pos):
    """TCP orientation for a horizontal side grasp.

    The tcp site's local +z points horizontally from the robot base toward the
    object, local +x points up, and local +y is the horizontal gripper closing
    axis. The site frame is inherited from link6 in the MJCF.
    """
    forward = np.array([target_pos[0], target_pos[1], 0.0], dtype=float)
    n = np.linalg.norm(forward)
    if n < 1e-6:
        forward = np.array([1.0, 0.0, 0.0])
    else:
        forward /= n
    up = np.array([0.0, 0.0, 1.0])
    closing_axis = np.cross(forward, up)
    closing_axis /= np.linalg.norm(closing_axis)
    return np.column_stack([up, closing_axis, forward])


def horizontal_pregrasp_point(grasp_pos, grasp_mat, distance=0.10):
    """Back away from the object along the horizontal approach direction."""
    approach_dir = grasp_mat[:, 2]
    return grasp_pos - distance * approach_dir


def _site_pose(data, site_id):
    T = np.eye(4)
    T[:3, :3] = data.site_xmat[site_id].reshape(3, 3)
    T[:3, 3] = data.site_xpos[site_id]
    return T


def ik_target_analytic(data, site_id, target_pos, target_mat=None):
    """Analytic-IK path: drive the tcp site to (target_pos, target_mat) with the
    shared DH solver piper_arm.solve_ik, so sim and real robot use ONE IK.

    Only valid on the DH-matching model (sim/models/piper_real), whose link6 body
    frame equals the DH FK to <0.1 mm. The constant rigid offset from the DH link6
    frame to the tcp site is read live from the current state:

        T_off = inv(DH_link6(q)) @ site_pose(q)          (constant; DH-base == world)

    so no calibration constant is baked in. The desired DH link6 pose is then
    T_dh6 = M_site @ inv(T_off), and solve_ik's target is the DH TCP (link6 + 0.06 z):

        T_target = M_site @ inv(T_off) @ translate(0, 0, 0.06)

    Position-only calls (target_mat is None) synthesize the horizontal grasp frame
    so the analytic solver still gets a full pose. Returns (q6, ok); ok=False lets
    the caller fall back to the MuJoCo numeric IK.
    """
    arm = _piper_arm()
    q_now = np.array(data.qpos[:6], dtype=float)
    T_dh6 = arm.forward_kinematics(q_now)                     # DH link6 in base(==world) frame
    T_off = np.linalg.inv(T_dh6) @ _site_pose(data, site_id)  # constant DH link6 -> tcp site
    inv_off = np.linalg.inv(T_off)
    R0 = horizontal_grasp_frame(target_pos) if target_mat is None else np.asarray(target_mat, float)

    def _solve_for(R):
        M_site = np.eye(4)
        M_site[:3, :3] = R
        M_site[:3, 3] = np.asarray(target_pos, float)
        T_dh6_des = M_site @ inv_off                          # desired DH link6 pose
        T_target = T_dh6_des.copy()
        T_target[:3, 3] = T_dh6_des[:3, 3] + 0.06 * T_dh6_des[:3, 2]  # DH TCP = link6 + 0.06 * z
        return arm.solve_ik(T_target, seed=list(q_now))       # warm-start from current arm q

    def _rot_z(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def _rot_y(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

    # A 2-finger gripper's closing axis is a line, so rolling 180 deg about the approach
    # axis is the SAME grasp. horizontal_grasp_frame points link6 +x up, which needs a
    # ~180 deg wrist flip the joint-6 limit forbids; the rolled variant points +x down and
    # is reachable. When an orientation is requested (grasp/lift) keep it exact -- only the
    # two rolls. For position-only calls (pre-grasp/carry: target_mat is None) the caller
    # does not constrain orientation, so also try a downward-pitched approach; that reaches
    # low/near poses (~0.32 m radius) the real Piper cannot reach purely horizontally.
    pitches = (0.0,) if target_mat is not None else (0.0, 0.26, -0.26, 0.52, -0.52, 0.79)
    best = None
    for pit in pitches:
        R_pit = R0 @ _rot_y(pit)                              # tilt approach axis
        for roll in (0.0, np.pi):
            sol = _solve_for(R_pit @ _rot_z(roll))
            if sol is not False:
                # Seed-INDEPENDENT cost: prefer a level approach and the wrist roll with the
                # smaller |joint6| (the non-flipped branch). A seed-dependent choice can pick
                # opposite rolls for the pre-grasp vs the grasp, flipping the wrist 180 deg
                # between them and sweeping the gripper through the object. A deterministic
                # criterion keeps the whole approach->grasp->lift sequence on one branch.
                cost = abs(pit) * 10.0 + abs(float(sol[5]))
                if best is None or cost < best[0]:
                    best = (cost, sol)
        if best is not None and target_mat is None and pit == 0.0:
            break                                             # prefer the level approach if it solves
    if best is None:
        return None, False
    return np.array(best[1], dtype=float), True


def ik_target_analytic_exact(data, site_id, target_pos, target_mat, seed=None):
    """Analytic branches only; unlike ``ik_target_analytic`` this never runs numeric IK."""
    arm = _piper_arm()
    q_now = np.asarray(seed if seed is not None else data.qpos[:6], float)
    T_dh6 = arm.forward_kinematics(np.asarray(data.qpos[:6], float))
    T_off = np.linalg.inv(T_dh6) @ _site_pose(data, site_id)
    sols = []
    for roll in (0.0, np.pi):
        c, s = np.cos(roll), np.sin(roll)
        local_roll = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        M_site = np.eye(4)
        M_site[:3, :3] = np.asarray(target_mat, float) @ local_roll
        M_site[:3, 3] = np.asarray(target_pos, float)
        T_dh6_des = M_site @ np.linalg.inv(T_off)
        target = T_dh6_des.copy()
        target[:3, 3] += 0.06 * target[:3, 2]
        sols.extend(arm.ik_analytic_all(target))
    if not sols:
        return None
    sols.sort(key=lambda q: np.linalg.norm(np.asarray(q) - q_now))
    return np.asarray(sols[0], float)


def ik_target(model, data, site_id, target_pos, target_mat=None, use_analytic=False):
    """IK from current state; returns arm q without disturbing sim.

    When ``use_analytic`` is set (the DH-matching piper_real model), solve with the
    shared analytic IK piper_arm.solve_ik so sim and real share one solver; if it
    reports no solution, fall back to the MuJoCo numeric IK below.

    When a horizontal side-grasp orientation is requested (``target_mat`` given)
    but the horizontal-axis constraint cannot be satisfied at this target, fall
    back to a position-only solve. Position-only IK converges reliably and at the
    natural arm pose the fingers already close roughly horizontally (DEVLOG D7),
    so the grasp still proceeds instead of aborting with "IK did not converge".
    """
    if use_analytic:
        q, ok = ik_target_analytic(data, site_id, target_pos, target_mat)
        if ok:
            return q, ok
        print("[ik] analytic IK found no solution; falling back to MuJoCo numeric IK")

    def _solve(horizontal):
        dik = mujoco.MjData(model)
        dik.qpos[:] = data.qpos
        dik.qvel[:] = 0
        return ik_mj.solve_ik(
            model,
            dik,
            site_id,
            target_pos,
            target_horizontal_axis=1 if horizontal else None,
            iters=1000,
            pos_tol=2e-3,
            rot_tol=0.15,
            lam=0.05,
            step=0.35,
        )

    if target_mat is None:
        return _solve(horizontal=False)
    q, ok = _solve(horizontal=True)
    if ok:
        return q, ok
    # Horizontal constraint infeasible here -> degrade to position-only so a
    # reachable (if not perfectly horizontal) grasp pose is still produced.
    print("[ik] horizontal-grasp IK infeasible; falling back to position-only")
    return _solve(horizontal=False)


def settle(model, data, ctrl, n, frames=None, renderer=None, cam="world_cam", every=20):
    for i in range(n):
        data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
        if frames is not None and renderer is not None and i % every == 0:
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.vis.global_.offwidth = 640
    model.vis.global_.offheight = 480
    data = mujoco.MjData(model)
    tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    cup = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cup")
    renderer = mujoco.Renderer(model, height=480, width=640)
    frames = []

    # Home + settle objects.
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    data.ctrl[:] = model.key_ctrl[home]
    settle(model, data, model.key_ctrl[home], 600)

    # Perception: confirm YOLO sees the cup (Route A still uses ground-truth pose).
    rgb = perception.render_rgb(model, data, "world_cam", 640, 480)
    boxes, _, _ = perception.detect(rgb)
    cup_seen = any(c == "cup" for c, _, _ in boxes)
    print(f"YOLO cup detected: {cup_seen}  ({[(c, round(cf,2)) for c,cf,_ in boxes]})")

    cup0 = data.xpos[cup].copy()
    grasp = cup0 + np.array([0, 0, 0.005])          # grasp center: cup body, slightly up
    grasp_mat = horizontal_grasp_frame(grasp)
    pre = horizontal_pregrasp_point(grasp, grasp_mat)  # pre-grasp: horizontal backoff
    lift = grasp + np.array([0, 0, 0.15])              # lift target
    print(f"cup0={cup0}  grasp={grasp}")

    ctrl = np.array(model.key_ctrl[home])

    # 1) pre-grasp (open)
    q, ok = ik_target(model, data, tcp, pre)
    ctrl[:6] = q; ctrl[6] = GRIP_OPEN
    settle(model, data, ctrl, 700, frames, renderer)
    print(f"pre-grasp  ik_ok={ok}  tcp={data.site_xpos[tcp]}")

    # 2) descend to grasp (open)
    q, ok = ik_target(model, data, tcp, grasp, grasp_mat)
    ctrl[:6] = q; ctrl[6] = GRIP_OPEN
    settle(model, data, ctrl, 700, frames, renderer)
    print(f"at-grasp   ik_ok={ok}  tcp={data.site_xpos[tcp]}  cup={data.xpos[cup]}")

    # 3) close gripper
    ctrl[6] = GRIP_CLOSE
    settle(model, data, ctrl, 600, frames, renderer)
    print(f"closed     gripper_qpos={data.qpos[6]:.4f}  cup={data.xpos[cup]}")

    # 4) lift (keep same arm target moving up); re-IK to lift point
    q, ok = ik_target(model, data, tcp, lift, grasp_mat)
    ctrl[:6] = q
    settle(model, data, ctrl, 900, frames, renderer)
    cup_now = data.xpos[cup].copy()
    print(f"lift       ik_ok={ok}  tcp={data.site_xpos[tcp]}  cup={cup_now}")

    rose = cup_now[2] - cup0[2]
    print(f"\ncup rose {rose*1000:.0f} mm  ->  {'LIFTED' if rose > 0.03 else 'NOT lifted'}")

    # save final frame + a montage of key moments
    Image.fromarray(frames[-1]).save(OUT / "phase5_final.png")
    idx = np.linspace(0, len(frames) - 1, 4).astype(int)
    W, H = 640, 480
    canvas = Image.new("RGB", (W * 2 + 6, H * 2 + 6), (20, 20, 20))
    for j, fi in enumerate(idx):
        r, c = j // 2, j % 2
        canvas.paste(Image.fromarray(frames[fi]), (c * (W + 6), r * (H + 6)))
    canvas.save(OUT / "phase5_grasp_montage.png")
    gif = [Image.fromarray(f) for f in frames[::2]]
    gif[0].save(OUT / "phase5_grasp.gif", save_all=True, append_images=gif[1:],
                duration=60, loop=0)
    print(f"saved {OUT/'phase5_grasp_montage.png'} and {OUT/'phase5_grasp.gif'}")


if __name__ == "__main__":
    main()
