#!/usr/bin/env python3
"""Direct Lumos SDK test of go_home() and set_joint() (planned motion)."""
import argparse
import math
import os
import sys
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--action', choices=('home', 'joint'), required=True)
    p.add_argument('--can', default='can0')
    p.add_argument('--sdk-root', type=Path)
    p.add_argument('--sdk-python-path', type=Path)
    p.add_argument('--delta-deg', nargs=6, type=float, default=[5, 0, 0, 0, 0, 0],
                   help='For joint: offset relative to actual current joints, degrees')
    p.add_argument('--time-sec', type=float, default=3.0)
    p.add_argument('--max-change-deg', type=float, default=15.0,
                   help='Conservative command-change guard; user must know actual limits')
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()
    if args.time_sec <= 0 or args.max_change_deg <= 0:
        p.error('time-sec/max-change-deg must be positive')
    if not all(math.isfinite(x) for x in args.delta_deg):
        p.error('delta-deg values must be finite')

    if args.sdk_python_path:
        sys.path.insert(0, str(args.sdk_python_path.expanduser().resolve()))
    from startouchclass import SingleArm
    if args.sdk_root:
        os.chdir(args.sdk_root.expanduser().resolve())

    arm = None
    try:
        print('SingleArm initialization may ENABLE motors. Keep E-stop accessible.')
        arm = SingleArm(can_interface_=args.can, gripper=False, enable_fd_=False)
        time.sleep(1.0)
        current = [float(v) for v in arm.get_joint_positions()]
        if len(current) != 6 or not all(math.isfinite(v) for v in current):
            raise RuntimeError('Invalid Lumos joint feedback')
        if args.action == 'home':
            goal = [0.0] * 6
        else:
            goal = [q + math.radians(d) for q, d in zip(current, args.delta_deg)]
        changes = [abs(math.degrees(q - c)) for q, c in zip(goal, current)]
        print('Current (rad):', [round(x, 3) for x in current])
        print('Goal    (rad):', [round(x, 3) for x in goal])
        print('Change  (deg):', [round(x, 1) for x in changes])
        if max(changes) > args.max_change_deg:
            raise RuntimeError('Change exceeds --max-change-deg guard. Check physical limits.')
        if not args.execute:
            print('PREVIEW ONLY. Add --execute to move Lumos.')
            return
        if input('Verify clear workspace and joint limits; type RUN: ').strip() != 'RUN':
            print('Cancelled.')
            return
        if args.action == 'home':
            print('Calling arm.go_home() (SDK uses 3 seconds).')
            arm.go_home()
            time.sleep(3.3)
        else:
            print(f'Calling arm.set_joint(goal, tf={args.time_sec}).')
            arm.set_joint(goal, tf=args.time_sec)
            time.sleep(args.time_sec + 0.3)
        actual = [float(v) for v in arm.get_joint_positions()]
        print('Actual  (rad):', [round(x, 3) for x in actual])
    finally:
        if arm is not None:
            arm.cleanup()


if __name__ == '__main__':
    main()
