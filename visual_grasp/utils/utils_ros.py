import time
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_matrix
from utils.utils_math import rotation_matrix_to_quaternion


def publish_tf(arm, joints, time):
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    """发布所有关节的TF变换"""
    for i in range(6):
        # 计算当前关节的变换矩阵
        T = arm.get_joint_tf(i, joints[i])

        # 创建TF消息
        transform_stamped = TransformStamped()
        # transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.stamp = time
        transform_stamped.header.frame_id = "link{}".format(i) if i > 0 else "base_link"
        transform_stamped.child_frame_id = "link{}".format(i + 1)

        # 提取平移和旋转
        transform_stamped.transform.translation.x = T[0, 3]
        transform_stamped.transform.translation.y = T[1, 3]
        transform_stamped.transform.translation.z = T[2, 3]

        # 转换旋转矩阵为四元数
        rot_matrix = T[:3, :3]
        quat = rotation_matrix_to_quaternion(rot_matrix)
        transform_stamped.transform.rotation.x = quat[1]
        transform_stamped.transform.rotation.y = quat[2]
        transform_stamped.transform.rotation.z = quat[3]
        transform_stamped.transform.rotation.w = quat[0]

        # 发布变换
        tf_broadcaster.sendTransform(transform_stamped)
        rospy.logdebug("Published transform: link{} -> link{}".format(i, i + 1))

    # pub gripper
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "link6"
    transform_stamped.child_frame_id = "gripper_base"

    transform_stamped.transform.translation.x = 0
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = 0

    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
        # 发布变换
    tf_broadcaster.sendTransform(transform_stamped)

    transform_stamped.header.frame_id = "gripper_base"
    transform_stamped.child_frame_id = "link7"
    transform_stamped.transform.translation.z = 0.1358
    transform_stamped.transform.rotation.x = 0.70711
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 0.70711
        # 发布变换
    tf_broadcaster.sendTransform(transform_stamped)

    transform_stamped.child_frame_id = "link8"
    transform_stamped.transform.translation.z = 0.1358
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0.70711
    transform_stamped.transform.rotation.z = 0.70711
    transform_stamped.transform.rotation.w = 0
        # 发布变换
    tf_broadcaster.sendTransform(transform_stamped)

    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "world"
    transform_stamped.child_frame_id = "base_link"

    transform_stamped.transform.translation.x = 0
    transform_stamped.transform.translation.y = 0
    transform_stamped.transform.translation.z = 0
    transform_stamped.transform.rotation.x = 0
    transform_stamped.transform.rotation.y = 0
    transform_stamped.transform.rotation.z = 0
    transform_stamped.transform.rotation.w = 1
        # 发布变换
    tf_broadcaster.sendTransform(transform_stamped)

    # publish camera
    transform_stamped.header.frame_id = "link6"
    transform_stamped.child_frame_id = "camera"
    transform_stamped.transform.translation.x = arm.link6_t_camera[0]
    transform_stamped.transform.translation.y = arm.link6_t_camera[1]
    transform_stamped.transform.translation.z = arm.link6_t_camera[2]
    transform_stamped.transform.rotation.x = arm.link6_q_camera[1]
    transform_stamped.transform.rotation.y = arm.link6_q_camera[2]
    transform_stamped.transform.rotation.z = arm.link6_q_camera[3]
    transform_stamped.transform.rotation.w = arm.link6_q_camera[0]
    # 发布变换
    tf_broadcaster.sendTransform(transform_stamped)



def publish_sphere_marker(pub, point, frame_id="base_link", color=(1.0, 0.0, 0.0, 1.0), radius=0.02):
    """
    发布三维点坐标的球形Marker
    参数:
    point (geometry_msgs.msg.Point): 输入点坐标
    frame_id (str): 坐标系ID，默认使用"map"
    color (tuple): RGBA颜色值(0-1范围)
    radius (float): 球体半径，默认0.1米
    """
    # 构建Marker消息
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "point_markers"
    marker.id = 0
    marker.action = Marker.ADD

    # 设置类型为球体
    marker.type = Marker.SPHERE

    # 设置位置和姿态
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = point[2]
    marker.pose.orientation.w = 1.0  # 默认无旋转

    # 设置尺寸
    marker.scale.x = radius * 2
    marker.scale.y = radius * 2
    marker.scale.z = radius * 2

    # 设置颜色
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    # 设置生命周期
    marker.lifetime = rospy.Duration(1.0)  # 持续显示1秒

    # 发布消息
    rospy.loginfo("发布球体Marker...")
    pub.publish(marker)

path = Path()
path.header.frame_id = "base_link"
path.poses = []

def publish_trajectory(path_pub, position):

    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "base_link"

    # 设置位置
    pose_stamped.pose.position.x = position[0]
    pose_stamped.pose.position.y = position[1]
    pose_stamped.pose.position.z = position[2]

    path.poses.append(pose_stamped)

    # 发布轨迹
    path_pub.publish(path)

def publish_target_point(pub, point, frame_id="camera"):
    point_msg = PointStamped()
    point_msg.header.stamp = rospy.Time.now()  # 当前时间戳
    point_msg.header.frame_id = frame_id  # 坐标系名称

    # 填充坐标数据
    point_msg.point.x = point[0]
    point_msg.point.y = point[1]
    point_msg.point.z = point[2]

    # 发布消息
    pub.publish(point_msg)

