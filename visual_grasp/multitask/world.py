"""SimWorld: a thin shared context around a loaded MuJoCo grasp scene.

Holds the model/data/renderer plus the current ctrl vector, and exposes the few
low-level motions the primitives need (settle, observe, move arm, drive gripper).
Everything is built on the existing sim/ helpers so this is a wrapper, not a
rewrite.

Headless: requires MUJOCO_GL=egl.
"""
import os
import pathlib
import sys
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import grasp_sim  # noqa: E402  (settle, GRIP_*)
from .control import MotionSafetyError

SCENE = ROOT / "sim" / "models" / "piper" / "scene_multitask.xml"
WRIST_OBSERVE_CTRL = np.array([0.2, 0.4, 0.0, 0.0, 0.0, 0.0, 0.035])


@dataclass(frozen=True)
class ClearanceSample:
    phase: str
    robot_geom: str
    table_clearance_m: float
    simulation_time_s: float


@dataclass(frozen=True)
class JointTrajectoryReport:
    steps: int
    max_following_error_rad: float
    min_table_clearance_m: float | None


def _annotate_yolo(img, labels, conf):
    """Draw YOLO target-class boxes (cup/bottle/bowl) on a rendered RGB frame."""
    try:
        import perception  # noqa: E402
    except ModuleNotFoundError as exc:
        if exc.name == "ultralytics":
            raise RuntimeError(
                "--annotate requires the YOLO stack (ultralytics). Run without "
                "--annotate when using sim_gt on a detector-free machine."
            ) from exc
        raise
    from PIL import Image, ImageDraw
    boxes, _, _ = perception.detect(img, conf=conf, labels=set(labels))
    if not boxes:
        return img
    pil = Image.fromarray(img)
    draw = ImageDraw.Draw(pil)
    for cls, c, (x1, y1, x2, y2) in boxes:
        color = (255, 220, 0)
        draw.rectangle([x1, y1, x2, y2], outline=color, width=3)
        draw.text((x1 + 2, max(0, y1 - 11)), f"{cls} {c:.2f}", fill=color)
    return np.asarray(pil)


@dataclass
class SimWorld:
    model: "mujoco.MjModel"
    data: "mujoco.MjData"
    renderer: "mujoco.Renderer"
    tcp: int
    ctrl: np.ndarray
    cam: str = "wrist_cam"
    frames: list = field(default_factory=list)
    record: bool = True
    record_cam: str = "world_cam"      # camera the evidence frames are rendered from
    annotate_labels: tuple = ()        # if set, draw YOLO boxes for these labels on frames
    annotate_conf: float = 0.25
    use_analytic_ik: bool = False      # drive IK with piper_arm.solve_ik (piper_real model)
    ready_ctrl: np.ndarray = None      # home/ready ctrl to reset to before each pick
    after_step: Callable[[], None] | None = None  # optional passive-viewer sync hook
    clearance_minima: dict = field(default_factory=dict)

    @classmethod
    def load(cls, scene=None, cam="wrist_cam", record=True,
             record_cam="world_cam", annotate_labels=(), annotate_conf=0.25,
             use_analytic_ik=False):
        """Load the scene, settle objects at the home pose, and return a SimWorld."""
        scene = pathlib.Path(scene) if scene else SCENE
        model = mujoco.MjModel.from_xml_path(str(scene))
        model.vis.global_.offwidth = 640
        model.vis.global_.offheight = 480
        data = mujoco.MjData(model)
        renderer = mujoco.Renderer(model, height=480, width=640) if record else None
        tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
        home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        ctrl = np.array(model.key_ctrl[home])
        world = cls(model, data, renderer, tcp, ctrl, cam, [], record,
                    record_cam, tuple(annotate_labels or ()), annotate_conf,
                    use_analytic_ik)
        world.ready_ctrl = np.array(model.key_ctrl[home])  # the home/ready pose to reset to
        data.ctrl[:] = ctrl
        # If a textured graspable bottle is present, move the placeholder green
        # 'bottle' far off-table so only the real, detectable bottle is in scene.
        green = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bottle")
        ycb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ycb_bottle")
        if green >= 0 and ycb >= 0:
            adr = model.jnt_qposadr[model.body_jntadr[green]]
            data.qpos[adr:adr + 3] = [2.0, 2.0, 0.1]
        world.settle(700)  # let the free objects settle on the table
        return world

    def settle(self, n, every=20):
        """Step the sim n times holding the current ctrl; record frames if enabled."""
        for i in range(n):
            self.data.ctrl[:] = self.ctrl
            mujoco.mj_step(self.model, self.data)
            if self.after_step is not None:
                self.after_step()
            if self.record and i % every == 0:
                self.frames.append(self._render_frame())

    def _robot_geom(self, geom_id):
        body = int(self.model.geom_bodyid[geom_id])
        while body > 0:
            name = (mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body) or "").lower()
            if any(token in name for token in
                   ("link", "gripper", "finger", "piper")):
                return True
            body = int(self.model.body_parentid[body])
        return False

    def _robot_table_contact(self):
        table = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
        if table < 0:
            return None
        for contact in self.data.contact:
            if table not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == table else contact.geom1
            if self._robot_geom(other):
                body = int(self.model.geom_bodyid[other])
                return (mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, body) or
                    f"body:{body}")
        return None

    def measure_table_clearance(self, phase="UNKNOWN") -> tuple[ClearanceSample, ...]:
        """Measure vertical mesh clearance over the finite table footprint.

        MuJoCo's generic mesh-box distance can return an ambiguous zero for
        separated meshes. Transforming collision-mesh vertices into the table
        frame gives a stable conservative vertical-clearance diagnostic for this
        horizontal tabletop scene.
        """
        table = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
        if table < 0:
            return ()
        table_pos = np.asarray(self.data.geom_xpos[table], float)
        table_rot = np.asarray(self.data.geom_xmat[table], float).reshape(3, 3)
        table_size = np.asarray(self.model.geom_size[table], float)
        samples = []
        for geom_id in range(self.model.ngeom):
            if (not self._robot_geom(geom_id) or
                    self.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH or
                    not (self.model.geom_contype[geom_id] or
                         self.model.geom_conaffinity[geom_id])):
                continue
            mesh_id = int(self.model.geom_dataid[geom_id])
            start = int(self.model.mesh_vertadr[mesh_id])
            count = int(self.model.mesh_vertnum[mesh_id])
            vertices = np.asarray(
                self.model.mesh_vert[start:start + count], float)
            geom_rot = np.asarray(
                self.data.geom_xmat[geom_id], float).reshape(3, 3)
            world_vertices = (vertices @ geom_rot.T +
                              np.asarray(self.data.geom_xpos[geom_id], float))
            table_local = (world_vertices - table_pos) @ table_rot
            over_table = ((np.abs(table_local[:, 0]) <= table_size[0] + 0.01) &
                          (np.abs(table_local[:, 1]) <= table_size[1] + 0.01))
            if not np.any(over_table):
                continue
            clearance = float(np.min(table_local[over_table, 2]) - table_size[2])
            body = int(self.model.geom_bodyid[geom_id])
            name = (mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body) or f"body:{body}")
            samples.append(ClearanceSample(
                phase, name, clearance, float(self.data.time)))
        return tuple(samples)

    def record_table_clearance(self, phase):
        samples = self.measure_table_clearance(phase)
        if not samples:
            return None
        sample = min(samples, key=lambda row: row.table_clearance_m)
        previous = self.clearance_minima.get(phase)
        if previous is None or sample.table_clearance_m < previous.table_clearance_m:
            self.clearance_minima[phase] = sample
        return sample

    def follow_joint_waypoints(
            self, waypoints, grip=None, max_joint_velocity_rad_s=0.8,
            max_following_error_rad=0.12, final_settle_steps=100,
            phase="TRANSIT", min_table_clearance_m=0.010):
        """Continuously move the position-control target along a joint polyline."""
        points = [np.asarray(point, float).copy() for point in waypoints]
        if len(points) < 2:
            raise ValueError("joint trajectory requires at least two waypoints")
        if max_joint_velocity_rad_s <= 0:
            raise ValueError("max_joint_velocity_rad_s must be positive")
        if max_following_error_rad <= 0:
            raise ValueError("max_following_error_rad must be positive")
        if grip is not None:
            self.ctrl[6] = float(grip)
        total_steps = 0
        max_error = 0.0
        min_clearance = None
        timestep = float(self.model.opt.timestep)
        for start, end in zip(points, points[1:]):
            duration = float(np.max(np.abs(end - start))) / max_joint_velocity_rad_s
            steps = max(1, int(np.ceil(duration / timestep)))
            for index in range(1, steps + 1):
                alpha = index / steps
                target = start + alpha * (end - start)
                self.ctrl[:6] = target
                self.data.ctrl[:] = self.ctrl
                mujoco.mj_step(self.model, self.data)
                total_steps += 1
                error = float(np.max(np.abs(self.data.qpos[:6] - target)))
                max_error = max(max_error, error)
                if error > max_following_error_rad:
                    raise MotionSafetyError(
                        f"{phase} following error {error:.4f} rad exceeds "
                        f"{max_following_error_rad:.4f} rad")
                contact_body = self._robot_table_contact()
                if contact_body is not None:
                    raise MotionSafetyError(
                        f"{phase} robot-table contact at {contact_body}")
                if total_steps % 5 == 0 or index == steps:
                    sample = self.record_table_clearance(phase)
                    if sample is not None:
                        min_clearance = (sample.table_clearance_m
                                         if min_clearance is None else
                                         min(min_clearance, sample.table_clearance_m))
                        if sample.table_clearance_m < min_table_clearance_m:
                            raise MotionSafetyError(
                                f"{phase} table clearance "
                                f"{sample.table_clearance_m * 1000:.1f} mm below "
                                f"{min_table_clearance_m * 1000:.1f} mm")
                if self.after_step is not None:
                    self.after_step()
                if self.record and total_steps % 20 == 0:
                    self.frames.append(self._render_frame())
        self.ctrl[:6] = points[-1]
        for index in range(int(final_settle_steps)):
            self.data.ctrl[:] = self.ctrl
            mujoco.mj_step(self.model, self.data)
            total_steps += 1
            if self._robot_table_contact() is not None:
                raise MotionSafetyError(f"{phase} robot-table contact while settling")
            if index % 5 == 0 or index + 1 == final_settle_steps:
                sample = self.record_table_clearance(phase)
                if sample is not None:
                    min_clearance = (sample.table_clearance_m
                                     if min_clearance is None else
                                     min(min_clearance, sample.table_clearance_m))
            if self.after_step is not None:
                self.after_step()
            if self.record and total_steps % 20 == 0:
                self.frames.append(self._render_frame())
        return JointTrajectoryReport(
            total_steps, round(max_error, 6), min_clearance)

    def _render_frame(self):
        if self.renderer is None:
            raise RuntimeError("recording is disabled; no renderer is available")
        self.renderer.update_scene(self.data, camera=self.record_cam)
        img = self.renderer.render().copy()
        if self.annotate_labels:
            img = _annotate_yolo(img, self.annotate_labels, self.annotate_conf)
        return img

    def observe(self, steps=900):
        """Move to the default eye-in-hand observation pose (frames the cup)."""
        self.move_to_pose(WRIST_OBSERVE_CTRL, steps)

    def move_to_pose(self, ctrl, steps=900):
        """Drive the full ctrl vector (arm + gripper) to a scan/observation pose."""
        self.ctrl[:] = np.asarray(ctrl, dtype=float)
        self.settle(steps)

    def set_arm(self, q, grip=None, steps=700):
        """Drive the 6 arm joints to q (and optionally the gripper) and settle."""
        self.ctrl[:6] = q
        if grip is not None:
            self.ctrl[6] = grip
        self.settle(steps)

    def gripper(self, value, steps=400):
        """Drive the gripper actuator (open ~0.035, close ~-0.005) and settle."""
        self.ctrl[6] = value
        self.settle(steps)

    def tcp_pos(self):
        return self.data.site_xpos[self.tcp].copy()

    def body_pos(self, name):
        """World position of a named body (ground truth; used only for verification)."""
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        return self.data.xpos[bid].copy() if bid >= 0 else None

    def body_z(self, name):
        """World z of a named body (ground truth; used only for grasp verification)."""
        pos = self.body_pos(name)
        return float(pos[2]) if pos is not None else None

    def collision_z_range(self, name):
        """(zmin, zmax) world-frame vertical extent of a body's active (collidable)
        geoms -- e.g. the actual rim height of a container, or how far a held
        object extends below its own center. Assumes each geom is z-axis-aligned
        with its body, true for every prop in this scene (same assumption
        measure_table_clearance already makes for the table)."""
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            return None
        zmin = zmax = None
        for gid in range(self.model.ngeom):
            if self.model.geom_bodyid[gid] != bid:
                continue
            if not (self.model.geom_contype[gid] or self.model.geom_conaffinity[gid]):
                continue  # decorative-only geom (contype=0 conaffinity=0), doesn't collide
            gtype = self.model.geom_type[gid]
            size = self.model.geom_size[gid]
            if gtype == mujoco.mjtGeom.mjGEOM_BOX:
                half_z = float(size[2])
            elif gtype in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
                half_z = float(size[1])
            elif gtype == mujoco.mjtGeom.mjGEOM_SPHERE:
                half_z = float(size[0])
            else:
                half_z = float(self.model.geom_rbound[gid])  # conservative fallback (mesh etc.)
            center_z = float(self.data.geom_xpos[gid][2])
            lo, hi = center_z - half_z, center_z + half_z
            zmin = lo if zmin is None else min(zmin, lo)
            zmax = hi if zmax is None else max(zmax, hi)
        return None if zmin is None else (zmin, zmax)
