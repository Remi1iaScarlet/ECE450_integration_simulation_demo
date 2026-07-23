#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster
import pyrealsense2 as rs
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from utils.utils_ros import publish_sphere_marker
from piper_sdk import *
import math
PI = math.pi
from piper_arm import PiperArm

# 根据棋盘格摆放位置测量
world_T_chessboard = np.array(
    [[0, 1, 0, 0.22],
     [1, 0, 0, 0.08],
     [0, 0, -1, 0.0],
     [0, 0, 0, 1]])
class ChessboardPublisher:
    def __init__(self):
        # 初始化参数
        self.pattern_size = (11, 8)  # 棋盘格内部角点数量
        self.square_size = 0.015  # 棋盘格方块实际尺寸（米）
        self.camera_matrix = []
        self.dist_coeffs = []
        # self.camera_matrix = np.load('/path/to/camera_matrix.npy')  # 加载标定参数
        # self.dist_coeffs = np.load('/path/to/dist_coeffs.npy')

        # 初始化ROS发布者
        self.pose_pub = rospy.Publisher('/camera_pose', PoseStamped, queue_size=10)
        self.point_pub = rospy.Publisher('/chessboard_point', PointCloud2, queue_size=10)
        self.base_corner_pub = rospy.Publisher('/measured_chessboard_origin', Marker, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster()

    def detect_and_publish(self, image, camera_matrix, dist_coeffs):

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

        # 转换为灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 检测棋盘格角点。SB 版本在透视畸变、光照不均时通常比经典方法稳。
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        if hasattr(cv2, "findChessboardCornersSB"):
            ret, corners = cv2.findChessboardCornersSB(gray, self.pattern_size, flags)
        else:
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FILTER_QUADS
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)

        if not ret:
            print("未检测到棋盘格！")
            return

        print("检测到棋盘格！")

        # 亚像素优化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # 绘制检测结果
        cv2.drawChessboardCorners(image, self.pattern_size, corners2, ret)
        debug_corners = corners2.reshape(-1, 2)
        for idx, label in [(0, "0"), (1, "1"), (self.pattern_size[0], "Y")]:
            p = debug_corners[idx].astype(int)
            cv2.circle(image, tuple(p), 8, (0, 0, 255), -1)
            cv2.putText(image, label, tuple(p + np.array([8, -8])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(image, f"Pattern: {self.pattern_size[0]}x{self.pattern_size[1]}",
                   (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        cv2.imshow('Chessboard Detection', image)
        cv2.waitKey(1)


        # 准备3D点坐标
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2) * self.square_size

        # 计算位姿
        success, rvec, tvec = cv2.solvePnP(objp, corners2, self.camera_matrix, self.dist_coeffs)

        if not success:
            print("位姿计算失败！")
            return

        projected, _ = cv2.projectPoints(objp, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        reproj_error = np.linalg.norm(projected.reshape(-1, 2) - corners2.reshape(-1, 2), axis=1)
        print("reprojection error mean/max px:", float(np.mean(reproj_error)), float(np.max(reproj_error)))

        # 转换为ROS消息格式
        self.publish_pose(rvec, tvec)
        self.publish_points(objp)

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        cam_T_chessboard = np.eye(4)
        cam_T_chessboard[:3, :3] = rotation_matrix
        cam_T_chessboard[0, 3] = tvec[0]
        cam_T_chessboard[1, 3] = tvec[1]
        cam_T_chessboard[2, 3] = tvec[2]

        link0_T_link6 = self.get_piper_fk()

        link6_T_cam = np.linalg.inv(link0_T_link6) @ world_T_chessboard @ np.linalg.inv(cam_T_chessboard)

        print(" #######################################")
        print("link6_T_cam\n", link6_T_cam)
        publish_sphere_marker(
            self.base_corner_pub,
            world_T_chessboard[:3, 3],
            frame_id="base_link",
            color=(1.0, 0.0, 1.0, 1.0),
            radius=0.015,
        )

    def get_piper_fk(self):
        piper = C_PiperInterface_V2()
        piper.ConnectPort()
        piper_arm = PiperArm()

        print(piper.GetArmJointMsgs())
        msg = piper.GetArmJointMsgs()

        theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
        theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
        theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
        theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
        theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
        theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0

        joints = [theta1, theta2, theta3, theta4, theta5, theta6]

        T_total = piper_arm.forward_kinematics(joints)

        return T_total
    def publish_pose(self, rvec, tvec):

        # 创建消息头
        now = rospy.Time.now()
        transform_stamped = geometry_msgs.msg.TransformStamped()
        pose_stamped = geometry_msgs.msg.PoseStamped()

        # 设置坐标系信息
        transform_stamped.header.stamp = now
        transform_stamped.header.frame_id = "camera_link"  # 参考坐标系
        transform_stamped.child_frame_id = "chessboard"  # 相机坐标系

        pose_stamped.header = transform_stamped.header
        pose_stamped.pose.position.x = tvec[0]
        pose_stamped.pose.position.y = tvec[1]
        pose_stamped.pose.position.z = tvec[2]

        # 转换旋转向量到四元数
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
        transform_stamped.transform.rotation.x = quaternion[1]
        transform_stamped.transform.rotation.y = quaternion[2]
        transform_stamped.transform.rotation.z = quaternion[3]
        transform_stamped.transform.rotation.w = quaternion[0]

        transform_stamped.transform.translation = pose_stamped.pose.position
        # 发布TF变换
        self.tf_broadcaster.sendTransform(transform_stamped)


        # chess_board --- world
        world_R_chessboard = np.array([[0, -1, 0],
                               [-1, 0, 0],
                                [0, 0, -1]]).T
        world_t_chessboard = [0, 0.23, 0]
        world_q_chessboard = self.rotation_matrix_to_quaternion(world_R_chessboard)
        transform_stamped.header.frame_id = "chessboard"
        transform_stamped.child_frame_id = "world"

        transform_stamped.transform.translation.x = world_t_chessboard[0]
        transform_stamped.transform.translation.y = world_t_chessboard[1]
        transform_stamped.transform.translation.z = world_t_chessboard[2]
        transform_stamped.transform.rotation.x = world_q_chessboard[1]
        transform_stamped.transform.rotation.y = world_q_chessboard[2]
        transform_stamped.transform.rotation.z = world_q_chessboard[3]
        transform_stamped.transform.rotation.w = world_q_chessboard[0]
        # 发布变换
        self.tf_broadcaster.sendTransform(transform_stamped)



        # 发布位姿消息
        self.pose_pub.publish(pose_stamped)


    def publish_points(self, objp):
        # 创建消息头
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "chessboard"

        # 定义点云字段
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        # 转换为ROS消息格式
        ros_cloud = pc2.create_cloud_xyz32(header, objp)

        ros_cloud.header.stamp = rospy.Time.now()
        self.point_pub.publish(ros_cloud)

    def rotation_matrix_to_quaternion(self, R):
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


def visualRGBD(color_image, depth_image):

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # Stack both images horizontally
    images = np.hstack((color_image, depth_colormap))
    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        return

if __name__ == '__main__':
    rospy.init_node('chessboard_publisher')

    publisher = ChessboardPublisher()

    # 配置RealSense管道
    pipeline = rs.pipeline()
    config = rs.config()

    # 启用对齐的深度和RGB流（关键：对齐到RGB流）
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 启动管道并获取对齐器
    profile = pipeline.start(config)
    align_to = rs.stream.color  # 对齐到RGB流
    align = rs.align(align_to)

    while not rospy.is_shutdown():
        # 等待同步的帧（深度+RGB）
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)  # 对齐深度到RGB

        # 提取对齐后的深度帧和原始RGB帧
        aligned_depth = aligned_frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # 检查帧有效性
        if not (aligned_depth and color_frame):
            continue

        # --------------------------
        # 1. 准备深度数据和相机内参
        # --------------------------
        depth_data = np.asanyarray(aligned_depth.get_data())
        depth_intrin = aligned_depth.profile.as_video_stream_profile().intrinsics  # 对齐后的内参（对应RGB视角）

        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        # 构建相机内参矩阵
        camera_matrix = np.array([
            [color_intrin.fx, 0, color_intrin.ppx],
            [0, color_intrin.fy, color_intrin.ppy],
            [0, 0, 1]
        ], dtype=np.float32)

        # 获取畸变系数（RealSense SDK中的畸变模型为Brown-Conrady）
        dist_coeffs = np.array(color_intrin.coeffs, dtype=np.float32)

        # print("camera_matrix \n", camera_matrix)
        # print("dist_coeffs \n", dist_coeffs)

        # --------------------------
        # 2. 准备RGB数据和颜色转换
        # --------------------------
        color_data = np.asanyarray(color_frame.get_data())
        # RealSense RGB默认是RGB格式（无需转换），若为BGR则用：color_data = color_data[..., ::-1]

        visualRGBD(color_data, depth_data)

        publisher.detect_and_publish(color_data, camera_matrix, dist_coeffs)
