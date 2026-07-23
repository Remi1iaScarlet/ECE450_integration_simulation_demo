#!/usr/bin/env python3
import argparse
import math
import time

from piper_sdk import C_PiperInterface_V2


def read_joint_degrees(piper, raw=False):
    msg = piper.GetArmJointMsgs()
    if raw:
        print(msg)
    joints = msg.joint_state
    return [
        joints.joint_1 * 1e-3,
        joints.joint_2 * 1e-3,
        joints.joint_3 * 1e-3,
        joints.joint_4 * 1e-3,
        joints.joint_5 * 1e-3,
        joints.joint_6 * 1e-3,
    ]


def main():
    parser = argparse.ArgumentParser(description="Read Piper current joint angles without enabling motion.")
    parser.add_argument("--can", default="can0", help="CAN interface name.")
    parser.add_argument("--samples", type=int, default=20, help="Number of feedback samples to read.")
    parser.add_argument("--interval", type=float, default=0.1, help="Seconds between samples.")
    parser.add_argument("--raw", action="store_true", help="Print raw SDK joint message for each sample.")
    args = parser.parse_args()

    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()

    deg = None
    print(f"Reading {args.samples} samples from {args.can}...")
    for i in range(args.samples):
        deg = read_joint_degrees(piper, raw=args.raw)
        print(f"sample {i + 1:02d}: " + " ".join(f"{v:.3f}" for v in deg))
        time.sleep(args.interval)

    if deg is None:
        raise RuntimeError("No joint samples were read.")

    rad = [math.radians(v) for v in deg]

    print("\nCurrent Piper joints, using last sample:")
    print("degrees:", " ".join(f"{v:.3f}" for v in deg))
    print("radians:", " ".join(f"{v:.6f}" for v in rad))
    print()
    print("Move-to-this-pose command:")
    print(
        "python3 scripts/move_piper_calib_pose.py --deg "
        + " ".join(f"{v:.3f}" for v in deg)
        + " --speed 8"
    )
    print()
    print("Enable-and-hold-current-pose command:")
    print("python3 scripts/enable_piper_current_pose.py --speed 8")


if __name__ == "__main__":
    main()
