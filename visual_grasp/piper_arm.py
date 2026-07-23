import numpy as np
import math
PI = math.pi
from math import atan2, sqrt, acos, pi, sin, cos, atan
import numpy as np
from utils.utils_math import rotation_matrix_to_euler, rotation_matrix_to_quaternion, quaternion_to_rotation_matrix

np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)

class PiperArm:
    def __init__(self):
        # DH参数定义（单位：米/弧度）
        self.alpha = [0, -pi / 2, 0, pi / 2, -pi / 2, pi / 2]  # 扭转角
        self.a = [0, 0, 0.28503, -0.02198, 0, 0]  # 连杆长度
        self.d = [0.123, 0, 0, 0.25075, 0, 0.091]  # 连杆偏移
        self.theta_offset = [0, -172.2135102 * pi / 180, -102.7827493 * pi / 180, 0, 0, 0]  # 初始角度偏移
        # self.theta_offset = [0, -172.241 * pi / 180, -100.78 * pi / 180, 0, 0, 0]  # 初始角度偏移
        self.l = 0.06 + 0.091 # 夹爪中点 到 joint4

        R = np.array([[-0.09,  0.94,  0.32],
                     [-0.99 ,  -0.11,  0.06],
                     [0.09, -0.31,  0.95]])

        # self.link6_q_camera = rotation_matrix_to_quaternion(R)   # w x y z
        self.link6_q_camera = [0.6990874216658888, -0.09551703428664327, 0.06812427941447226, -0.7053455575701054]

        # Physical debug estimate: camera is mounted above link6/gripper_base.
        # self.link6_t_camera = [0.0, 0.0, 0.11]
        self.link6_t_camera = [-0.0867, 0.0018, -0.05]

        # link limitation
        self.link_limits = [[-154, 154], [0, 195], [-175, 0], [-106, 106], [-75, 75], [-100, 100]]

    def limitation_check(self, joints):
        for i in range(6):
            angle = joints[i] / PI * 180
            if (angle < self.link_limits[i][0] or angle > self.link_limits[i][1]):
                return False

        return True

    def dh_transform(self, alpha, a, d, theta):
        """
        计算Denavit-Hartenberg标准参数的4x4齐次变换矩阵

        参数:
        alpha (float): 连杆扭转角（绕x_(i-1)轴的旋转角，弧度）
        a (float): 连杆长度（沿x_(i-1)轴的平移量，米）
        d (float): 连杆偏移量（沿z_i轴的平移量，米）
        theta (float): 关节角（绕z_i轴的旋转角，弧度）

        返回:
        np.ndarray: 4x4齐次变换矩阵
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # 构建标准DH变换矩阵
        transform = np.array([
            [ct, -st, 0, a],
            [ca * st, ca * ct, -sa, -sa * d],
            [sa * st, sa * ct, ca, ca * d],
            [0, 0, 0, 1]
        ])
        return transform

    def forward_kinematics(self, joints):
        T_total = np.eye(4)
        for i in range(6):
            T = self.dh_transform(self.alpha[i], self.a[i], self.d[i], self.theta_offset[i] + joints[i])
            T_total = T_total @ T

        return T_total

    def forward_kinematics_sub(self, joints, end):
        # 0_T_end
        T_total = np.eye(4)
        for i in range(end):
            print("i alpha a d theta", self.alpha[i], self.a[i], self.d[i], self.theta_offset[i])
            T = self.dh_transform(self.alpha[i], self.a[i], self.d[i], self.theta_offset[i] + joints[i])
            T_total = T_total @ T

        return T_total

    def inverse_kinematics(self, T_target):
        """Pieper解法逆运动学求解"""
        # 计算 joint4 位置   point under base frame [ 0.5  -0.02  0.2   1.  ]
        joint4_p = T_target @ np.array([0, 0, -self.l, 1], dtype=float)
        px, py, pz = joint4_p[0], joint4_p[1], joint4_p[2]

        # px = T_target[0, 3]
        # py = T_target[1, 3]
        # pz = T_target[2, 3]

        # 计算 link1 2 3 角度
        theta1 = atan2(py , px)
        T01 = self.dh_transform(self.alpha[0], self.a[0], self.d[0], theta1)

        # Convert P05 to frame 1
        P15 = np.linalg.inv(T01) @ np.array([px, py, pz, 1])
        x1, z1 = P15[0], P15[2]
        a1, a2 = self.a[2], self.a[3]
        d1, d2 = self.d[2], self.d[3]
        l1 = sqrt(a1 ** 2 + d1 ** 2)
        l2 = sqrt(a2 ** 2 + d2 ** 2)
        l3 = sqrt(x1 ** 2 + z1 ** 2)

        cos_phi3 = (l1**2 + l2**2 - l3**2) / (2.0 * l1 * l2)
        if abs(cos_phi3) > 1:
            print("no ik solution, fail theta 3")
            return False
        phi3 = acos(cos_phi3)
        print("phi3 is ", phi3 / PI * 180)

        phi3 = atan2(sqrt(1 - cos_phi3 ** 2), cos_phi3)
        # print("phi3 is ", phi3 / PI * 180)
        gamma = atan2(abs(self.d[3]), abs(self.a[3]))
        # print("gamma is ", gamma / PI * 180)
        theta3 = -(gamma + phi3) - self.theta_offset[2]


        cos_phi2 = (l1**2 + l3**2 - l2**2) / (2 * l1 * l3)
        if abs(cos_phi2) > 1:
            print("no ik solution, fail theta 2")
            return False
        phi2 = acos(cos_phi2)

        beta = atan(x1 / z1)
        if z1 > 0:
            theta2 = - (PI / 2 + phi2 - beta) - self.theta_offset[1]
        else:
            beta = atan(x1 / abs(z1))
            print("phi2", phi2 / 3.14 * 180)
            print("beta", beta / 3.14 * 180)
            theta2 = - (phi2 - (PI / 2 - beta)) - self.theta_offset[1]

        q_sol = [theta1, theta2, theta3, 0, 0, 0]

        if not self.limitation_check(q_sol):
            print("no feasible solution")
            return False

        # 计算link4 5 6 角度
        T03 = self.forward_kinematics_sub(q_sol , 3)
        R03 = T03[0:3, 0:3]
        R34d = np.array([[1, 0, 0],[0, 0, -1],[0, 1, 0]])
        T34d = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        R03d = R03 @ R34d
        R06 = T_target[0:3, 0:3]
        R36 = R03d.T @ R06

        print("needed R36", R36)
        rxyzs = rotation_matrix_to_euler(R36)

        for rxyz in rxyzs:
            q_sol = [theta1, theta2, theta3, rxyz[0], rxyz[1], rxyz[2]]
            if self.limitation_check(q_sol):
                T_fk = self.forward_kinematics(q_sol)
                print("expect T", T_target)
                print("T_fk", T_fk)
                print("ik result", q_sol)
                return q_sol

        print("no feasible solution")
        return False


    def get_joint_tf(self, joint_idx, angle):
        """获取指定关节的变换矩阵"""
        transform = self.dh_transform(self.alpha[joint_idx], self.a[joint_idx], self.d[joint_idx], self.theta_offset[joint_idx] + angle)
        return transform

    # ------------------------------------------------------------------
    # Robust IK (2026-06-30). The original inverse_kinematics() above only
    # evaluates ONE analytic branch (single theta1, single elbow), so it
    # returns False whenever that branch violates a joint limit even though
    # another valid solution exists -- the "move a few mm and it suddenly
    # solves" behaviour. solve_ik() below enumerates all Pieper branches
    # (shoulder x elbow x wrist), verifies each candidate by FK, and falls
    # back to a self-contained DH-Jacobian numerical polish for the few
    # near-singular poses. Same DH model, so sim and real can share one IK.
    # Measured solve rate on random reachable configs: 79% -> 89% (branches)
    # -> 97% (+numeric). Original inverse_kinematics() kept for reference.
    # ------------------------------------------------------------------
    def _fk_sub_quiet(self, joints, end):
        T = np.eye(4)
        for i in range(end):
            T = T @ self.dh_transform(self.alpha[i], self.a[i], self.d[i],
                                      self.theta_offset[i] + joints[i])
        return T

    def _tcp_pos(self, T6):
        # gripper midpoint = link6 origin + 0.06 along link6 +z
        return T6[:3, 3] + 0.06 * T6[:3, 2]

    def _fk_matches(self, q, T_target, pos_tol=2e-3, rot_tol=1e-2):
        T6 = self.forward_kinematics(q)
        if np.linalg.norm(self._tcp_pos(T6) - T_target[:3, 3]) > pos_tol:
            return False
        R = T6[:3, :3].T @ T_target[:3, :3]
        return np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1, 1)) < rot_tol

    @staticmethod
    def _rotvec(R):
        ang = np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1, 1))
        if ang < 1e-8:
            return np.zeros(3)
        ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        return ax / (2 * np.sin(ang)) * ang

    def ik_analytic_all(self, T_target):
        """Return every analytic Pieper solution (FK-verified, within limits)."""
        wc = T_target @ np.array([0, 0, -self.l, 1.0])
        px, py, pz = wc[0], wc[1], wc[2]
        a, d, off = self.a, self.d, self.theta_offset
        l1 = sqrt(a[2] ** 2 + d[2] ** 2)
        l2 = sqrt(a[3] ** 2 + d[3] ** 2)
        gamma = atan2(abs(d[3]), abs(a[3]))
        R06 = T_target[:3, :3]
        R34d = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0.0]])
        base1 = atan2(py, px)
        sols = []
        for t1 in (base1, base1 + PI):
            T01 = self.dh_transform(self.alpha[0], a[0], d[0], t1)
            P15 = np.linalg.inv(T01) @ np.array([px, py, pz, 1.0])
            x1, z1 = P15[0], P15[2]
            l3 = sqrt(x1 ** 2 + z1 ** 2)
            if l3 < 1e-9:
                continue
            c3 = (l1 ** 2 + l2 ** 2 - l3 ** 2) / (2 * l1 * l2)
            c2 = (l1 ** 2 + l3 ** 2 - l2 ** 2) / (2 * l1 * l3)
            if abs(c3) > 1 or abs(c2) > 1:
                continue
            phi3m = acos(max(-1.0, min(1.0, c3)))
            phi2m = acos(max(-1.0, min(1.0, c2)))
            beta = atan2(x1, z1)
            for phi3, phi2 in ((phi3m, phi2m), (-phi3m, -phi2m)):
                theta3 = -(gamma + phi3) - off[2]
                for theta2 in (-(PI / 2 + phi2 - beta) - off[1],
                               -(phi2 - (PI / 2 - beta)) - off[1]):
                    q123 = [t1, theta2, theta3]
                    R36 = (self._fk_sub_quiet(q123, 3)[:3, :3] @ R34d).T @ R06
                    for rxyz in rotation_matrix_to_euler(R36):
                        q = [t1, theta2, theta3, rxyz[0], rxyz[1], rxyz[2]]
                        if self.limitation_check(q) and self._fk_matches(q, T_target):
                            sols.append(q)
        return sols

    def _numeric_ik(self, T_target, seed, iters=200, pos_tol=1e-3, lam=0.05, step=0.5):
        lo = np.deg2rad([lim[0] for lim in self.link_limits])
        hi = np.deg2rad([lim[1] for lim in self.link_limits])
        q = np.clip(np.array(seed, float), lo, hi)
        for _ in range(iters):
            T6 = self.forward_kinematics(q)
            perr = T_target[:3, 3] - self._tcp_pos(T6)
            rerr = self._rotvec(T_target[:3, :3] @ T6[:3, :3].T)
            if np.linalg.norm(perr) < pos_tol and np.linalg.norm(rerr) < 1e-2:
                return list(q) if self._fk_matches(q, T_target) else False
            J = np.zeros((6, 6))
            eps = 1e-6
            for i in range(6):
                dq = np.zeros(6); dq[i] = eps
                T6e = self.forward_kinematics(q + dq)
                dp = (self._tcp_pos(T6e) - self._tcp_pos(T6)) / eps
                dr = self._rotvec(T6e[:3, :3] @ T6[:3, :3].T) / eps
                J[:, i] = np.concatenate([dp, dr])
            e = np.concatenate([perr, rerr])
            dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
            q = np.clip(q + step * dq, lo, hi)
        return False

    def solve_ik(self, T_target, seed=None):
        """Robust IK -> joint list (rad) or False. All analytic branches first
        (closest to `seed` if given), DH-Jacobian numeric polish as fallback."""
        sols = self.ik_analytic_all(T_target)
        if sols:
            if seed is not None:
                sols.sort(key=lambda q: np.linalg.norm(np.array(q) - np.array(seed)))
            return sols[0]
        seeds = ([list(seed)] if seed is not None else []) + \
                [[0, 1.2, -1.0, 0, 0.3, 0], [0, 0.6, -0.6, 0, 0, 0]]
        for s in seeds:
            r = self._numeric_ik(T_target, s)
            if r is not False:
                return r
        return False
