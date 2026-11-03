#!/usr/bin/env python3
"""Direct Lumos SDK test of go_home() and set_joint() (planned motion)."""
import argparse
from contextlib import contextmanager
import math
import os
import time
from pathlib import Path


def find_sdk_root() -> Path | None:
    """Find the Lumos SDK config root without requiring a command-line path."""
    script_dir = Path(__file__).resolve().parent
    candidates = (
        script_dir / ".dependencies" / "startouch_sdk",
        script_dir / "startouch_sdk",
        Path.cwd(),
        Path.cwd() / "startouch_sdk",
    )
    for candidate in candidates:
        root = candidate.expanduser().resolve()
        if (root / "src" / "config" / "robot_kinematics.yaml").is_file():
            return root
    return None


@contextmanager
def sdk_initialization_directory(root: Path | None):
    original = Path.cwd()
    try:
        if root is not None:
            os.chdir(root)
        yield
    finally:
        os.chdir(original)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--action', choices=('home', 'joint'), required=True)
    p.add_argument('--can', default='can0')
    p.add_argument('--delta-deg', nargs=6, type=float, default=[5, 0, 0, 0, 0, 0],
                   help='For joint: offset relative to actual current joints, degrees')
    p.add_argument('--time-sec', type=float, default=3.0)
    p.add_argument('--max-change-deg', type=float, default=60.0,
                   help='Conservative command-change guard; user must know actual limits')
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()
    if args.time_sec <= 0 or args.max_change_deg <= 0:
        p.error('time-sec/max-change-deg must be positive')
    if not all(math.isfinite(x) for x in args.delta_deg):
        p.error('delta-deg values must be finite')

    sdk_root = find_sdk_root()
    try:
        from startouchclass import SingleArm
    except ImportError as exc:
        raise RuntimeError(
            f'Cannot import Lumos Python SDK: {exc}. Install startouchclass and its dependencies '
            'in the active Python environment.'
        ) from exc

    arm = None
    try:
        print('SingleArm initialization may ENABLE motors. Keep E-stop accessible.')
        if sdk_root is not None:
            print(f'Lumos SDK config: {sdk_root / "src/config/robot_kinematics.yaml"}')
        else:
            print('Lumos SDK config root not found in standard locations; trying current environment.')
        with sdk_initialization_directory(sdk_root):
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
        if input('Verify clear workspace and joint limits; type r to execute: ').strip().lower() != 'r':
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
