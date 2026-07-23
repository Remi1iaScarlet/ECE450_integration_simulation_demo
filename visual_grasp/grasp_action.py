#!/usr/bin/env python3
from piper_sdk import *
import rospy
import time
import sys
import numpy as np
import math
from piper_arm import PiperArm
from utils.utils_piper import read_joints
from utils.utils_piper import enable_fun
from utils.utils_ros import publish_tf, publish_sphere_marker, publish_trajectory
from utils.utils_math import quaternion_to_rotation_matrix
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path

PI = math.pi
factor = 1000 * 180 / PI
receive_object_center = False
object_center = []

GRIP_OPEN = 0.10
GRIP_CLOSE = 0.045
PREGRASP_DISTANCE = 0.05
GRASP_Z_OFFSET = 0.0
CAMERA_GRASP_X_OFFSET = 0.02
LIFT_HEIGHT = 0.12
HOME_JOINTS = [0, 0, 0, 0, 0, 0, GRIP_CLOSE]
FIXED_GRASP_R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=float)
USE_HORIZONTAL_GRASP = False
IK_YAW_OFFSETS = [0, 15, -15, 30, -30, 45, -45]
IK_ROLL_OFFSETS = [0, 90, -90, 180]
LIFT_HEIGHT_CANDIDATES = [0.12, 0.08, 0.05]


def control_arm(joints, speed=2):

    # joints [rad]

    position = joints

    joint_0 = int(position[0] * factor)
    joint_1 = int(position[1] * factor)
    joint_2 = int(position[2] * factor)
    joint_3 = int(position[3] * factor)
    joint_4 = int(position[4] * factor)
    joint_5 = int(position[5] * factor)

    if (joint_4 < -70000) :
        joint_4 = -70000

    # piper.MotionCtrl_1()
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

    if len(joints) > 6:
        joint_6 = round(position[6] * 1000 * 1000)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

    print(piper.GetArmStatus())
    print(position)

def object_point_callback(msg):
    # print("Receive visual detection result", msg.point.x, msg.point.y, msg.point.z)
    if(np.isnan(msg.point.x) or np.isnan(msg.point.y) or np.isnan(msg.point.z)):
        return
    global receive_object_center, object_center
    receive_object_center = True
    object_center = [msg.point.x, msg.point.y, msg.point.z]


def horizontal_grasp_frame(target_pos):
    """Side-grasp TCP frame migrated from the MuJoCo closed-loop demo."""
    forward = np.array([target_pos[0], target_pos[1], 0.0], dtype=float)
    norm = np.linalg.norm(forward)
    if norm < 1e-6:
        forward = np.array([1.0, 0.0, 0.0])
    else:
        forward /= norm

    up = np.array([0.0, 0.0, 1.0])
    closing_axis = np.cross(forward, up)
    closing_axis /= np.linalg.norm(closing_axis)
    return np.column_stack([up, closing_axis, forward])


def horizontal_pregrasp_point(grasp_pos, grasp_mat, distance=PREGRASP_DISTANCE):
    approach_dir = grasp_mat[:, 2]
    return grasp_pos - distance * approach_dir


def make_target_transform(position, rotation):
    target = np.eye(4, dtype=float)
    target[:3, :3] = rotation
    target[:3, 3] = position
    return target


def rotate_z(vec, degrees):
    theta = degrees * PI / 180.0
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)
    return rot @ vec


def frame_from_tcp_z(z_axis, roll_degrees=0):
    z_axis = np.array(z_axis, dtype=float)
    z_axis /= np.linalg.norm(z_axis)

    x_ref = np.array([0.0, 0.0, -1.0])
    if abs(np.dot(x_ref, z_axis)) > 0.95:
        x_ref = np.array([0.0, 1.0, 0.0])
    x_axis = x_ref - np.dot(x_ref, z_axis) * z_axis
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    roll = roll_degrees * PI / 180.0
    x_roll = np.cos(roll) * x_axis + np.sin(roll) * y_axis
    y_roll = -np.sin(roll) * x_axis + np.cos(roll) * y_axis
    return np.column_stack([x_roll, y_roll, z_axis])


def candidate_ik_rotations(position, nominal_rotation):
    rotations = [FIXED_GRASP_R]
    if USE_HORIZONTAL_GRASP:
        rotations.append(nominal_rotation)

    radial = np.array([position[0], position[1], 0.0], dtype=float)
    if np.linalg.norm(radial) < 1e-6:
        radial = np.array([1.0, 0.0, 0.0])
    radial /= np.linalg.norm(radial)

    for yaw in IK_YAW_OFFSETS:
        z_axis = rotate_z(radial, yaw)
        for roll in IK_ROLL_OFFSETS:
            rotations.append(frame_from_tcp_z(z_axis, roll))

    unique = []
    for rotation in rotations:
        if not any(np.allclose(rotation, existing, atol=1e-6) for existing in unique):
            unique.append(rotation)
    return unique


def solve_grasp_ik(piper_arm, position, rotation, label):
    for idx, candidate_rotation in enumerate(candidate_ik_rotations(position, rotation)):
        target = make_target_transform(position, candidate_rotation)
        joints = piper_arm.inverse_kinematics(target)
        if joints:
            print(label, "ik candidate", idx, "[degree]:", np.array(joints) / PI * 180)
            return joints

    print(label, "IK failed for all candidate wrist poses at", position)
    return False


def solve_lift_ik(piper_arm, grasp, grasp_mat):
    for height in LIFT_HEIGHT_CANDIDATES:
        lift = grasp + np.array([0.0, 0.0, height])
        lift_joints = solve_grasp_ik(piper_arm, lift, grasp_mat, "lift %.0fmm" % (height * 1000))
        if lift_joints:
            return lift_joints, lift
    return False, grasp + np.array([0.0, 0.0, LIFT_HEIGHT_CANDIDATES[-1]])


def execute_joint_target(joints, gripper, speed=20, wait_s=3.0):
    command = list(joints) + [gripper]
    control_arm(command, speed)
    time.sleep(wait_s)


def move_and_grasp(object_center, joints, piper_arm):
    print("prepare to grasp point under camera frame", object_center[0], object_center[1], object_center[2])
    adjusted_object_center = np.array(object_center, dtype=float)
    adjusted_object_center[0] += CAMERA_GRASP_X_OFFSET
    print("adjusted grasp point under camera frame", adjusted_object_center)

    # transfer point from camera frame to base_link frame
    base_T_link6 = piper_arm.forward_kinematics(joints)
    link6_T_cam = np.eye(4)
    link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
    link6_T_cam[:3, 3] = piper_arm.link6_t_camera

    base_ob_center = base_T_link6 @ link6_T_cam @ np.append(adjusted_object_center, 1.0)

    # publish target object center
    print("point under base frame", base_ob_center)
    pub = rospy.Publisher('/target_point_under_based', Marker, queue_size=10)
    publish_sphere_marker(pub, base_ob_center, frame_id="base_link", color=(0.0, 1.0, 0.0, 1.0), radius=0.02)

    grasp = base_ob_center[:3].copy()
    grasp[2] += GRASP_Z_OFFSET
    grasp_mat = horizontal_grasp_frame(grasp)
    pre = horizontal_pregrasp_point(grasp, grasp_mat)
    lift = grasp + np.array([0.0, 0.0, LIFT_HEIGHT_CANDIDATES[0]])
    print("base grasp/pre/lift", grasp, pre, lift)

    pre_joints = solve_grasp_ik(piper_arm, pre, grasp_mat, "pre-grasp")
    grasp_joints = solve_grasp_ik(piper_arm, grasp, grasp_mat, "grasp")
    lift_joints, lift = solve_lift_ik(piper_arm, grasp, grasp_mat)
    failed = []
    if not pre_joints:
        failed.append("pre-grasp")
    if not grasp_joints:
        failed.append("grasp")
    if not lift_joints:
        failed.append("lift")
    if failed:
        print("ik fail at", failed, "skip this detection")
        return False

    # time_now = rospy.Time.now()
    # publish_tf(piper_arm, joints, time_now)

    execute_joint_target(pre_joints, GRIP_OPEN, speed=20, wait_s=4.0)
    execute_joint_target(grasp_joints, GRIP_OPEN, speed=15, wait_s=4.0)
    execute_joint_target(grasp_joints, GRIP_CLOSE, speed=10, wait_s=2.0)
    execute_joint_target(lift_joints, GRIP_CLOSE, speed=15, wait_s=4.0)
    control_arm(HOME_JOINTS, 20)

    return True



if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    piper.GripperCtrl(0, 1000, 0x01, 0)

    # 设置初始位置
    joints = HOME_JOINTS
    control_arm(joints, 100)
    time.sleep(2)


    # 初始化节点
    rospy.init_node('vison_grasp_node', anonymous=True)

    piper_arm = PiperArm()
    sub = rospy.Subscriber('/object_point',
                           PointStamped,
                           object_point_callback,
                           queue_size=10,
                           tcp_nodelay=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # time_now = rospy.Time.now()
        # publish_tf(piper_arm, joints, time_now)
        if (receive_object_center):
            msg = piper.GetArmJointMsgs()

            theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
            theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
            theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
            theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
            theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
            theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0

            joints = [theta1, theta2, theta3, theta4, theta5, theta6]

            if move_and_grasp(object_center, joints, piper_arm):
                break
            receive_object_center = False

        rate.sleep()


