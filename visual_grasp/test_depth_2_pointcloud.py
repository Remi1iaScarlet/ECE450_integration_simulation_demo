#!/usr/bin/env python
import rospy
import pyrealsense2 as rs
import struct
import cv2
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from utils.utils_ros import publish_target_point, publish_sphere_marker
import numpy as np


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


def main():
    # 初始化ROS节点
    rospy.init_node('realsense_rgb_pointcloud_publisher')
    pub = rospy.Publisher('/camera/depth/color/points', PointCloud2, queue_size=10)

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

    try:
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

            # --------------------------
            # 2. 准备RGB数据和颜色转换
            # --------------------------
            color_data = np.asanyarray(color_frame.get_data())
            # RealSense RGB默认是RGB格式（无需转换），若为BGR则用：color_data = color_data[..., ::-1]

            visualRGBD(color_data, depth_data)

            # --------------------------
            # 3. 生成3D点坐标（基于对齐的深度图）
            # --------------------------
            height, width = depth_data.shape
            u = np.arange(width)  # 列索引（0~639）
            v = np.arange(height)  # 行索引（0~479）
            u_grid, v_grid = np.meshgrid(u, v)  # 生成坐标网格

            # 深度转米（RealSense深度单位是毫米）
            z = depth_data.astype(np.float32) / 1000.0

            # 相机内参投影（从像素坐标到3D坐标）
            fx, fy = depth_intrin.fx, depth_intrin.fy
            cx, cy = depth_intrin.ppx, depth_intrin.ppy
            x = (u_grid - cx) * z / fx
            y = (v_grid - cy) * z / fy

            # --------------------------
            # 4. 过滤无效点（深度为0或过大）
            # --------------------------
            valid_mask = (z > 0.1) & (z < 10.0)  # 保留0.1~10米内的点
            x_valid = x[valid_mask]
            y_valid = y[valid_mask]
            z_valid = z[valid_mask]
            color_valid = color_data[v_grid[valid_mask], u_grid[valid_mask]]  # 对应RGB颜色

            # --------------------------
            # 5. 构建ROS PointCloud2消息
            # --------------------------
            # 定义点云字段（x,y,z为浮点，rgb为打包的32位整数）
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.FLOAT32, 1)  # 打包的RGB（0xRRGGBB）
            ]

            # 构造点数据数组（x,y,z,rgb）
            num_points = len(x_valid)
            dtype = np.dtype([
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('rgb', np.float32)
            ])
            point_array = np.zeros(num_points, dtype=dtype)

            # 填充坐标数据
            point_array['x'] = x_valid
            point_array['y'] = y_valid
            point_array['z'] = z_valid

            # 填充颜色数据（将RGB字节打包为0xRRGGBB的浮点数）
            b = color_valid[:, 0].astype(np.uint32)
            g = color_valid[:, 1].astype(np.uint32)
            r = color_valid[:, 2].astype(np.uint32)
            rgb_packed = (r << 16) | (g << 8) | b  # 组合成0xRRGGBB格式
            point_array['rgb'] = rgb_packed.view(np.float32)

            # 转换为ROS PointCloud2消息
            header = rospy.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "camera"  # RGB相机坐标系

            cloud_msg = PointCloud2()
            cloud_msg.header = header
            cloud_msg.height = 1  # 无组织点云（单层）
            cloud_msg.width = num_points
            cloud_msg.fields = fields
            cloud_msg.is_bigendian = False
            cloud_msg.point_step = dtype.itemsize  # 每个点的字节数（16字节）
            cloud_msg.row_step = cloud_msg.point_step * num_points
            cloud_msg.data = point_array.tobytes()
            cloud_msg.is_dense = True  # 无无效点（已过滤）

            # 发布点云
            pub.publish(cloud_msg)

    finally:
        pipeline.stop()


if __name__ == '__main__':
    main()