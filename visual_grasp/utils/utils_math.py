import math
PI = math.pi
import numpy as np
from math import acos, atan2, sqrt, acos, pi, sin, cos

def rotation_matrix_to_euler(R):
    """从旋转矩阵计算欧拉角(ZYZ顺序) 返回两组解"""
    sin_theta = sqrt(R[2, 0] ** 2 + R[2, 1] ** 2)
    singular = sin_theta < 1e-6

    if not singular:
        theta = atan2(sin_theta, R[2, 2])
        phi = atan2(R[1, 2]/sin(theta), R[0, 2]/sin(theta))
        psi = atan2(R[2, 1]/sin(theta), -R[2, 0]/sin(theta))

        theta2 = -theta
        phi2 = atan2(R[1, 2]/sin(theta2), R[0, 2]/sin(theta2))
        psi2 = atan2(R[2, 1]/sin(theta2), -R[2, 0]/sin(theta2))
        return np.array([[phi, theta, psi], [phi2, theta2, psi2]])

    else:
        theta = 0
        phi = 0
        psi = atan2(-R[0, 1], R[0, 0])
        return np.array([[phi, theta, psi], [phi, theta, psi]])




def rotation_matrix_to_quaternion(R):
    """将3x3旋转矩阵转换为四元数(w, x, y, z顺序)"""
    q = np.zeros(4)
    trace = np.trace(R)

    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
        q[0] = 0.25 * S
        q[1] = (R[2, 1] - R[1, 2]) / S
        q[2] = (R[0, 2] - R[2, 0]) / S
        q[3] = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q[0] = (R[2, 1] - R[1, 2]) / S
        q[1] = 0.25 * S
        q[2] = (R[0, 1] + R[1, 0]) / S
        q[3] = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q[0] = (R[0, 2] - R[2, 0]) / S
        q[1] = (R[0, 1] + R[1, 0]) / S
        q[2] = 0.25 * S
        q[3] = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q[0] = (R[1, 0] - R[0, 1]) / S
        q[1] = (R[0, 2] + R[2, 0]) / S
        q[2] = (R[1, 2] + R[2, 1]) / S
        q[3] = 0.25 * S

    return q / np.linalg.norm(q)  # 归一化


def quaternion_to_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    # Extract the values from Q [w x y z]
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix