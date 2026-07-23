#!/usr/bin/env python3

from piper_sdk import *
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys
import time
import math
PI = math.pi
from piper_arm import PiperArm
import utils.utils_ros
from utils.utils_ros import publish_tf


if __name__ == "__main__":

    # 初始化节点
    rospy.init_node('piper_tf_publisher', anonymous=True)

    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    piper_arm = PiperArm()

    max = 100
    n = 1
    while not rospy.is_shutdown():

        print(piper.GetArmJointMsgs())
        msg = piper.GetArmJointMsgs()

        theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
        theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
        theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
        theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
        theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
        theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0

        joints = [theta1, theta2, theta3, theta4, theta5, theta6]
        print("thetas", joints)

        # joints = [-0.07120793326312327,
        #  1.7892262456478087,
        #  -0.7524313475828899,
        #  0, # 3.0541009350061423,
        #  -0.9514063476513819,
        #  0# -3.0907142835747465#
        #            ]
        #
        # joints = [ item / max * n for item in joints ]

        time_now = rospy.Time.now()
        publish_tf(piper_arm, joints, time_now)

        time.sleep(0.1)
        n=n+1
        n =n % max
        pass
