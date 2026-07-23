from piper_sdk import *
import time
import math
PI = math.pi

def enable_fun(piper: C_PiperInterface_V2):
    '''
    使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
    '''
    enable_flag = False
    # 设置超时时间（秒）
    timeout = 5
    # 记录进入循环前的时间
    start_time = time.time()
    elapsed_time_flag = False
    while not (enable_flag):
        elapsed_time = time.time() - start_time
        print("--------------------")
        enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                      piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        print("使能状态:", enable_flag)
        piper.EnableArm(7)
        piper.GripperCtrl(0, 1000, 0x01, 0)
        print("--------------------")
        # 检查是否超过超时时间
        if elapsed_time > timeout:
            print("超时....")
            elapsed_time_flag = True
            enable_flag = True
            break
        time.sleep(1)
        pass
    if (elapsed_time_flag):
        print("程序自动使能超时,退出程序")
        exit(0)

def read_joints(piper):
    msg = piper.GetArmJointMsgs()
    print(msg)

    theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
    theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
    theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
    theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
    theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
    theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0

    joints = [theta1, theta2, theta3, theta4, theta5, theta6]
    return joints