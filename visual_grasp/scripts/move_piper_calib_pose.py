#!/usr/bin/env python3
import argparse
import math
import time

from piper_sdk import C_PiperInterface_V2
from utils.utils_piper import enable_fun, read_joints


FACTOR = 1000 * 180 / math.pi


def send_joint_pose(piper, joints_rad, speed):
    joint_cmd = [int(v * FACTOR) for v in joints_rad]
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(*joint_cmd)


def main():
    parser = argparse.ArgumentParser(
        description="Move Piper to a fixed calibration pose and keep the process alive."
    )
    parser.add_argument(
        "--deg",
        nargs=6,
        type=float,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        required=True,
        help="Target joint angles in degrees.",
    )
    parser.add_argument("--can", default="can0", help="CAN interface name.")
    parser.add_argument("--speed", type=int, default=10, help="Piper speed value. Use a low value for calibration.")
    parser.add_argument("--gripper", type=float, default=0.10, help="Gripper opening in meters.")
    parser.add_argument("--settle", type=float, default=8.0, help="Seconds to wait after sending the pose.")
    parser.add_argument(
        "--no-hold",
        action="store_true",
        help="Exit after reaching the pose instead of printing current joints repeatedly.",
    )
    args = parser.parse_args()

    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper)

    target_rad = [math.radians(v) for v in args.deg]
    gripper_cmd = round(abs(args.gripper) * 1000 * 1000)

    print("Moving Piper to calibration pose:")
    print("degrees:", args.deg)
    print("radians:", target_rad)
    print("speed:", args.speed)

    piper.GripperCtrl(gripper_cmd, 1000, 0x01, 0)
    send_joint_pose(piper, target_rad, args.speed)
    time.sleep(args.settle)

    print("Current joints after settle:")
    print(read_joints(piper))

    if args.no_hold:
        return

    print("Holding process. Keep this terminal open during calibration. Ctrl-C to stop.")
    while True:
        time.sleep(2.0)
        print("current joints rad:", read_joints(piper))


if __name__ == "__main__":
    main()
