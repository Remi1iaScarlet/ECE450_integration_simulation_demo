#!/usr/bin/env python3
import argparse
import math
import time

from piper_sdk import C_PiperInterface_V2
from utils.utils_piper import enable_fun


FACTOR = 1000 * 180 / math.pi


def read_joint_radians(piper):
    msg = piper.GetArmJointMsgs()
    joints = msg.joint_state
    return [
        math.radians(joints.joint_1 * 1e-3),
        math.radians(joints.joint_2 * 1e-3),
        math.radians(joints.joint_3 * 1e-3),
        math.radians(joints.joint_4 * 1e-3),
        math.radians(joints.joint_5 * 1e-3),
        math.radians(joints.joint_6 * 1e-3),
    ]


def send_joint_pose(piper, joints_rad, speed):
    joint_cmd = [int(v * FACTOR) for v in joints_rad]
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*joint_cmd)


def main():
    parser = argparse.ArgumentParser(
        description="Enable Piper and command it to hold the joint pose it is currently in."
    )
    parser.add_argument("--can", default="can0", help="CAN interface name.")
    parser.add_argument("--speed", type=int, default=8, help="Low speed value used when sending the hold pose.")
    parser.add_argument("--gripper", type=float, default=0.10, help="Gripper opening in meters.")
    parser.add_argument("--hold-period", type=float, default=2.0, help="Seconds between hold commands.")
    parser.add_argument("--no-loop", action="store_true", help="Send the hold command once and exit.")
    args = parser.parse_args()

    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()

    current = read_joint_radians(piper)
    print("Read current pose before enabling:")
    print("degrees:", " ".join(f"{math.degrees(v):.3f}" for v in current))

    piper.EnableArm(7)
    enable_fun(piper)
    piper.GripperCtrl(round(abs(args.gripper) * 1000 * 1000), 1000, 0x01, 0)

    print("Sending hold command at current pose.")
    send_joint_pose(piper, current, args.speed)

    if args.no_loop:
        return

    print("Holding current pose. Keep this terminal open during calibration. Ctrl-C to stop.")
    while True:
        send_joint_pose(piper, current, args.speed)
        time.sleep(args.hold_period)


if __name__ == "__main__":
    main()
