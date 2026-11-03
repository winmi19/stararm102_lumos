#!/usr/bin/env python3
"""StarArm102 leader -> Lumos: 50 Hz set_joint_raw teleop, optional gripper."""
import argparse
import math
import os
import sys
import time
from pathlib import Path

# IMPORTANT: Verify these six signs against your real mechanism before --execute.
SIGNS = (-1, 1, 1, 1, -1, -1)
LEADER_IDS = (0, 1, 2, 3, 4, 5)
HZ = 50
MAX_STEP_DEG = 1.0
MAX_OFFSET_DEG = 30.0


def read_leader(bus, gripper=False, button=False):
    ids = list(LEADER_IDS) + ([6] if gripper else []) + ([7] if button else [])
    feedback = bus.send_sync_servo_monitor(ids, realtime=True)
    values = []
    for sid in ids:
        entry = feedback.get(sid)
        angle = None if entry is None else entry.angle_monitor
        if angle is None or not math.isfinite(float(angle)):
            raise RuntimeError(f"Missing/invalid StarArm servo {sid} feedback")
        values.append(float(angle))
    return values


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--leader-port', default='/dev/ttyUSB0')
    p.add_argument('--can', default='can0')
    p.add_argument('--sdk-root', type=Path, help='Lumos SDK root for robot_kinematics.yaml')
    p.add_argument('--sdk-python-path', type=Path, help='Folder containing startouchclass.py')
    p.add_argument('--gripper', action='store_true', help='Enable leader servo 6 -> Lumos gripper')
    p.add_argument('--invert-gripper', action='store_true')
    p.add_argument('--hd-button', action='store_true', help='Use leader button board ID 7')
    p.add_argument('--execute', action='store_true', help='Actually send Lumos commands')
    args = p.parse_args()

    import serial
    import fashionstar_uart_sdk as uservo
    if args.sdk_python_path:
        sys.path.insert(0, str(args.sdk_python_path.expanduser().resolve()))
    from startouchclass import SingleArm

    if args.sdk_root:
        os.chdir(args.sdk_root.expanduser().resolve())

    uart = None
    arm = None
    try:
        uart = serial.Serial(args.leader_port, baudrate=1_000_000, timeout=0)
        bus = uservo.UartServoManager(uart)
        if args.hd_button and not bus.ping(7):
            raise RuntimeError('HD button ID 7 not detected; omit --hd-button if absent')
        print('Lumos SingleArm initialization may ENABLE motors. Keep E-stop ready.')
        arm = SingleArm(can_interface_=args.can, gripper=args.gripper, enable_fd_=False)
        time.sleep(1.0)
        if args.gripper:
            input('Place LEADER gripper fully CLOSED, then press Enter...')
            g_closed = read_leader(bus, gripper=True, button=args.hd_button)[6]
            input('Place LEADER gripper fully OPEN, then press Enter...')
            g_open = read_leader(bus, gripper=True, button=args.hd_button)[6]
            if abs(g_open - g_closed) < 2:
                raise RuntimeError('Gripper calibration range is <2 degrees')
            print(f'Gripper calibration: closed={g_closed:.1f}, open={g_open:.1f}')

        input('Align leader and Lumos to matching safe reference poses; Enter to capture...')
        leader_ref = read_leader(bus, gripper=args.gripper, button=args.hd_button)[:6]
        lumos_ref = [float(v) for v in arm.get_joint_positions()]
        if len(lumos_ref) != 6 or not all(math.isfinite(v) for v in lumos_ref):
            raise RuntimeError('Invalid Lumos joint feedback')
        print('Leader ref (deg):', [round(x, 2) for x in leader_ref])
        print('Lumos ref (rad):', [round(x, 3) for x in lumos_ref])
        print('Signs:', SIGNS, '; max offset:', MAX_OFFSET_DEG, 'deg')
        print('Mode:', 'LIVE MOTION' if args.execute else 'PREVIEW (no follower commands)')
        if args.execute and input('Type RUN to enable continuous follower commands: ').strip() != 'RUN':
            return

        previous = lumos_ref[:]
        last = time.monotonic()
        last_print = 0.0
        button_locked = None
        while True:
            started = time.monotonic()
            vals = read_leader(bus, gripper=args.gripper, button=args.hd_button)
            if args.hd_button:
                locked = vals[-1] == 180.0
                if locked != button_locked:
                    for sid in LEADER_IDS:
                        bus.stop_on_control_mode(sid, 0x11 if locked else 0x10, 0x00)
                    button_locked = locked
                    print('\nLeader HOLD' if locked else '\nLeader FREE')
            offsets = [SIGNS[i] * (vals[i] - leader_ref[i]) for i in range(6)]
            if any(abs(x) > MAX_OFFSET_DEG for x in offsets):
                raise RuntimeError('Leader offset exceeded 30 deg. Stopping commands.')
            desired = [lumos_ref[i] + math.radians(offsets[i]) for i in range(6)]
            # Bound per-cycle step so noisy samples do not create sudden targets.
            dt = max(0.001, min(started - last, 0.04))
            last = started
            step = math.radians(min(MAX_STEP_DEG, 30.0 * dt))
            target = [old + max(-step, min(step, new - old))
                      for old, new in zip(previous, desired)]
            if args.execute:
                arm.set_joint_raw(target, [0.0] * 6)
            previous = target
            grip = None
            if args.gripper:
                grip = max(0.0, min(1.0, (vals[6] - g_closed) / (g_open - g_closed)))
                if args.invert_gripper:
                    grip = 1.0 - grip
                if args.execute:
                    arm.setGripperPosition(grip)
            if started - last_print > 0.5:
                print('leader:', [round(x, 1) for x in vals[:6]],
                      'target_rad:', [round(x, 3) for x in target],
                      'gripper:', None if grip is None else round(grip, 2))
                last_print = started
            time.sleep(max(0.0, 1.0 / HZ - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        if arm is not None:
            arm.cleanup()
        if uart is not None:
            uart.close()


if __name__ == '__main__':
    main()
