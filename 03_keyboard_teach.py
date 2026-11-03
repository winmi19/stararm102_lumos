#!/usr/bin/env python3
"""StarArm 102 keyboard teaching with Lumos step-follow or batch replay."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

LEADER_JOINT_IDS = (0, 1, 2, 3, 4, 5)
LEADER_GRIPPER_ID = 6
JOINT_SIGNS = (-1.0, 1.0, 1.0, 1.0, -1.0, -1.0)


def read_leader(bus, use_gripper: bool = False):
    """Read six leader joints and, optionally, leader gripper servo 6."""
    ids = list(LEADER_JOINT_IDS)
    if use_gripper:
        ids.append(LEADER_GRIPPER_ID)
    feedback = bus.send_sync_servo_monitor(ids, realtime=True)
    values = []
    for servo_id in ids:
        item = feedback.get(servo_id)
        angle = None if item is None else item.angle_monitor
        if angle is None or not math.isfinite(float(angle)):
            raise RuntimeError(f"Missing/invalid StarArm feedback for servo {servo_id}")
        values.append(float(angle))
    return values


def map_joint_to_lumos(leader_deg, leader_ref_deg, lumos_ref_rad, max_offset_deg):
    """Map leader joint offsets (deg) onto Lumos reference joints (rad)."""
    offsets = [JOINT_SIGNS[i] * (leader_deg[i] - leader_ref_deg[i]) for i in range(6)]
    if any(abs(value) > max_offset_deg for value in offsets):
        raise ValueError(
            f"Leader offset exceeds {max_offset_deg:.1f} deg: "
            f"{[round(value, 1) for value in offsets]}"
        )
    return [lumos_ref_rad[i] + math.radians(offsets[i]) for i in range(6)]


def map_gripper_to_lumos(angle_deg, closed_deg, opened_deg, invert=False):
    """Convert leader gripper servo angle into Lumos normalized [0, 1]."""
    if abs(opened_deg - closed_deg) < 2.0:
        raise ValueError("Leader gripper calibration range is too small")
    ratio = (angle_deg - closed_deg) / (opened_deg - closed_deg)
    ratio = max(0.0, min(1.0, ratio))
    return 1.0 - ratio if invert else ratio


def check_leader_step(current, previous, max_jump_deg):
    jumps = [abs(a - b) for a, b in zip(current, previous)]
    if any(value > max_jump_deg for value in jumps):
        raise ValueError(
            f"Recorded point jumps more than {max_jump_deg:.1f} deg: "
            f"{[round(value, 1) for value in jumps]}"
        )


def check_lumos_path(reference, targets, max_jump_deg):
    previous = reference
    for index, target in enumerate(targets, 1):
        jumps = [abs(math.degrees(a - b)) for a, b in zip(target, previous)]
        if any(value > max_jump_deg for value in jumps):
            raise RuntimeError(
                f"Lumos target {index} jumps more than {max_jump_deg:.1f} deg: "
                f"{[round(value, 1) for value in jumps]}"
            )
        previous = target


def load_lumos(args):
    if args.sdk_python_path:
        sys.path.insert(0, str(args.sdk_python_path.expanduser().resolve()))
    if args.sdk_root:
        os.chdir(args.sdk_root.expanduser().resolve())
    from startouchclass import SingleArm
    # The SDK constructor may enable the real Lumos motors, including preview runs.
    return SingleArm(can_interface_=args.can, gripper=args.gripper, enable_fd_=False)


def save_json(path: Path, data):
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")


def validate_six(values, label):
    values = [float(value) for value in values]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"Invalid {label}: expected six finite values")
    return values


def print_points(records):
    if not records:
        print("No teaching points recorded.")
        return
    for index, record in enumerate(records, 1):
        grip = record.get("gripper_position")
        suffix = "" if grip is None else f", gripper={grip:.2f}"
        print(f"{index:02d}: {[round(v, 1) for v in record['leader_joints_deg']]} deg{suffix}")


def run_gripper_timeline(arm, gripper_points, seconds_per_segment, rate_hz, stop_event, errors):
    """Approximate gripper playback when native joint+gripper sync is unavailable."""
    try:
        if len(gripper_points) < 2:
            return
        total_segments = len(gripper_points) - 1
        total_duration = seconds_per_segment * total_segments
        start = time.monotonic()
        period = 1.0 / rate_hz
        while not stop_event.is_set():
            elapsed = min(time.monotonic() - start, total_duration)
            segment = min(int(elapsed / seconds_per_segment), total_segments - 1)
            fraction = (elapsed - segment * seconds_per_segment) / seconds_per_segment
            value = gripper_points[segment] + (
                gripper_points[segment + 1] - gripper_points[segment]
            ) * fraction
            arm.setGripperPosition(float(max(0.0, min(1.0, value))))
            if elapsed >= total_duration:
                break
            stop_event.wait(period)
        if not stop_event.is_set():
            arm.setGripperPosition(float(gripper_points[-1]))
    except Exception as exc:  # propagate worker-thread errors to the main thread
        errors.append(exc)
        stop_event.set()


def execute_batch(arm, full_path, gripper_path, args):
    """Execute one joint waypoint batch, with gripper support when requested."""
    requested_duration = args.seconds_per_segment * (len(full_path) - 1)
    started = time.monotonic()
    worker = None
    stop_event = threading.Event()
    worker_errors = []

    try:
        if not args.gripper:
            returned_duration = arm.set_joint_waypoints(full_path, time_sec=requested_duration)
        elif hasattr(arm.arm, "move_joint_waypoints_with_gripper"):
            print("Using native Lumos joint-waypoint + gripper synchronized API.")
            returned_duration = arm.move_joint_waypoints_with_gripper(
                full_path, gripper_path, time_sec=requested_duration
            )
        else:
            print(
                "WARNING: Native synchronized gripper waypoint API is unavailable. "
                "Using a Python-timed gripper interpolation alongside joint waypoints; "
                "the two trajectories are approximate, not hardware-time synchronized."
            )
            worker = threading.Thread(
                target=run_gripper_timeline,
                args=(arm, gripper_path, args.seconds_per_segment,
                      args.gripper_rate_hz, stop_event, worker_errors),
                daemon=True,
            )
            worker.start()
            returned_duration = arm.set_joint_waypoints(full_path, time_sec=requested_duration)
    except Exception:
        stop_event.set()
        if worker is not None:
            worker.join(timeout=1.0)
        raise

    if worker is not None:
        worker.join(timeout=requested_duration + 2.0)
        if worker.is_alive():
            stop_event.set()
            worker.join(timeout=1.0)
            raise RuntimeError("Gripper playback thread did not finish in time")
        if worker_errors:
            raise RuntimeError(f"Gripper playback failed: {worker_errors[0]}")

    planned_duration = requested_duration
    if returned_duration is not None:
        try:
            value = float(returned_duration)
            if math.isfinite(value) and value > 0:
                planned_duration = value
        except (TypeError, ValueError):
            pass
    remaining = planned_duration - (time.monotonic() - started)
    if remaining > 0:
        time.sleep(remaining)
    time.sleep(0.4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("step", "batch"), default="step",
                        help="step: each leader keypress immediately triggers follower; batch: teach all, then replay")
    parser.add_argument("--leader-port", default="/dev/ttyUSB0")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--sdk-root", type=Path,
                        help="Lumos SDK root containing src/config/robot_kinematics.yaml")
    parser.add_argument("--sdk-python-path", type=Path,
                        help="Directory containing startouchclass.py")
    parser.add_argument("--gripper", action="store_true",
                        help="Also teach StarArm servo 6 and control the Lumos gripper")
    parser.add_argument("--invert-gripper", action="store_true",
                        help="Invert the leader-to-Lumos gripper opening direction")
    parser.add_argument("--release-leader", action="store_true",
                        help="Explicitly unload leader joints 0..5 for hand-guiding")
    parser.add_argument("--seconds-per-segment", type=float, default=3.0)
    parser.add_argument("--max-offset-deg", type=float, default=30.0)
    parser.add_argument("--max-jump-deg", type=float, default=25.0)
    parser.add_argument("--gripper-rate-hz", type=float, default=20.0,
                        help="Python fallback gripper interpolation rate in batch mode")
    parser.add_argument("--save", type=Path, default=Path("stararm_lumos_teach.json"))
    parser.add_argument("--execute", action="store_true",
                        help="Actually command Lumos; without it, only preview targets")
    args = parser.parse_args()

    if (args.seconds_per_segment <= 0 or args.max_offset_deg <= 0
            or args.max_jump_deg <= 0 or args.gripper_rate_hz <= 0):
        parser.error("time, safety limits, and gripper rate must be positive")

    import serial
    import fashionstar_uart_sdk as uservo

    uart = None
    arm = None
    records = []
    data = {
        "created_at": datetime.now().astimezone().isoformat(),
        "mode": args.mode,
        "gripper_enabled": args.gripper,
        "joint_signs": list(JOINT_SIGNS),
        "leader_port": args.leader_port,
        "can_interface": args.can,
        "records": records,
    }
    save_path = args.save.expanduser().resolve()

    try:
        uart = serial.Serial(args.leader_port, baudrate=1_000_000, timeout=0)
        bus = uservo.UartServoManager(uart)

        if args.release_leader:
            print("WARNING: unloading leader torque can make the arm fall. Support it first.")
            if input("Type RELEASE to unload StarArm joints 0..5: ").strip() == "RELEASE":
                for servo_id in LEADER_JOINT_IDS:
                    bus.stop_on_control_mode(servo_id, 0x10, 0x00)
            else:
                print("Leader torque was not released.")

        gripper_closed = None
        gripper_opened = None
        if args.gripper:
            input("Move StarArm gripper to FULLY CLOSED, then press Enter...")
            gripper_closed = read_leader(bus, use_gripper=True)[-1]
            input("Move StarArm gripper to FULLY OPEN, then press Enter...")
            gripper_opened = read_leader(bus, use_gripper=True)[-1]
            if abs(gripper_opened - gripper_closed) < 2.0:
                raise RuntimeError("StarArm gripper calibration span is less than 2 degrees")
            print(f"Leader gripper calibration: closed={gripper_closed:.1f}, open={gripper_opened:.1f} deg")

        input("Place StarArm at its neutral/reference pose, then press Enter...")
        leader_reference = read_leader(bus)[:6]
        data["leader_reference_deg"] = leader_reference
        data["leader_gripper_closed_deg"] = gripper_closed
        data["leader_gripper_open_deg"] = gripper_opened
        print("Leader reference (deg):", [round(value, 2) for value in leader_reference])

        if args.mode == "batch":
            print("\nBATCH MODE: move the leader freely between points; follower will replay after teaching ends.")
            print("Commands: r=record, d=delete last, p=print points, q=finish teaching")
            while True:
                command = input("teach> ").strip().lower()
                if command == "r":
                    sample = read_leader(bus, use_gripper=args.gripper)
                    leader_joints = sample[:6]
                    previous = records[-1]["leader_joints_deg"] if records else leader_reference
                    try:
                        map_joint_to_lumos(leader_joints, leader_reference, [0.0] * 6,
                                           args.max_offset_deg)
                        check_leader_step(leader_joints, previous, args.max_jump_deg)
                    except ValueError as exc:
                        print("Rejected:", exc)
                        continue
                    grip = None
                    if args.gripper:
                        grip = map_gripper_to_lumos(sample[6], gripper_closed,
                                                    gripper_opened, args.invert_gripper)
                    records.append({"leader_joints_deg": leader_joints, "gripper_position": grip})
                    data["records"] = records
                    save_json(save_path, data)
                    print(f"Recorded point {len(records)}:", [round(value, 1) for value in leader_joints],
                          "gripper=" + ("off" if grip is None else f"{grip:.2f}"))
                elif command == "d":
                    if records:
                        removed = records.pop()
                        save_json(save_path, data)
                        print("Deleted last point:", [round(value, 1) for value in removed["leader_joints_deg"]])
                    else:
                        print("No point to delete.")
                elif command == "p":
                    print_points(records)
                elif command == "q":
                    break
                else:
                    print("Use r / d / p / q")
        else:
            print("\nSTEP MODE: move leader to a waypoint, then press r; Lumos follows that point immediately.")
            print("Commands: r=record and follow, p=print points, q=finish")

        if args.mode == "batch" and not records:
            print("No teaching points recorded; exiting.")
            return

        if args.mode == "batch":
            print("\nLeader teaching is finished. Move Lumos to the desired corresponding reference pose.")
            input("Check workspace and E-stop; press Enter to initialize Lumos...")
        else:
            print("\nSTEP MODE: align Lumos to the corresponding reference pose before starting.")
            input("Check workspace and E-stop; press Enter to initialize Lumos...")

        print("WARNING: SingleArm initialization may enable Lumos motors, even in preview mode.")
        arm = load_lumos(args)
        time.sleep(1.0)
        lumos_reference = validate_six(arm.get_joint_positions(), "Lumos joint feedback")
        data["lumos_reference_rad"] = lumos_reference
        print("Lumos reference (rad):", [round(value, 4) for value in lumos_reference])

        lumos_gripper_reference = None
        if args.gripper:
            lumos_gripper_reference = float(arm.get_gripper_position())
            if not math.isfinite(lumos_gripper_reference):
                raise RuntimeError("Invalid Lumos gripper feedback")
            data["lumos_gripper_reference"] = lumos_gripper_reference
            print(f"Lumos gripper reference: {lumos_gripper_reference:.3f}")

        if args.execute:
            print("EXECUTE mode: Lumos will move after each accepted teaching point." if args.mode == "step"
                  else "EXECUTE mode: Lumos will replay the recorded path.")
            if input("Clear workspace, verify limits and E-stop; type RUN: ").strip() != "RUN":
                print("Cancelled; no Lumos motion command sent.")
                return
        else:
            print("PREVIEW mode: no Lumos motion commands will be sent (controller initialization may still enable motors).")

        if args.mode == "step":
            print("\nCommands: r=record current leader pose and trigger follower, p=print, q=finish")
            previous_leader = leader_reference
            expected_lumos = lumos_reference[:]
            while True:
                command = input("step> ").strip().lower()
                if command == "r":
                    sample = read_leader(bus, use_gripper=args.gripper)
                    leader_joints = sample[:6]
                    try:
                        map_joint_to_lumos(leader_joints, leader_reference, lumos_reference,
                                           args.max_offset_deg)
                        check_leader_step(leader_joints, previous_leader, args.max_jump_deg)
                    except ValueError as exc:
                        print("Rejected:", exc)
                        continue
                    target = map_joint_to_lumos(leader_joints, leader_reference,
                                                lumos_reference, args.max_offset_deg)
                    grip = None
                    if args.gripper:
                        grip = map_gripper_to_lumos(sample[6], gripper_closed,
                                                    gripper_opened, args.invert_gripper)
                    record = {"leader_joints_deg": leader_joints,
                              "lumos_target_rad": target,
                              "gripper_position": grip}
                    records.append(record)
                    data["records"] = records
                    save_json(save_path, data)
                    print(f"Point {len(records)} Lumos target (rad):", [round(value, 4) for value in target])
                    if grip is not None:
                        print(f"Lumos gripper target: {grip:.3f}")
                    if args.execute:
                        actual_before = validate_six(arm.get_joint_positions(), "Lumos joint feedback")
                        if any(abs(a - b) > 0.15 for a, b in zip(actual_before, expected_lumos)):
                            raise RuntimeError("Lumos is not at the previous commanded point; aborting step mode")
                        if args.gripper:
                            arm.setGripperPosition(grip)
                        arm.set_joint(target, tf=args.seconds_per_segment)
                        time.sleep(args.seconds_per_segment + 0.35)
                        actual = validate_six(arm.get_joint_positions(), "Lumos joint feedback")
                        error = max(abs(a - b) for a, b in zip(actual, target))
                        print(f"Follower finished point {len(records)}; max joint error={error:.3f} rad")
                        if error > 0.15:
                            raise RuntimeError("Lumos did not reach the target within 0.15 rad; stopping")
                        expected_lumos = target
                    previous_leader = leader_joints
                elif command == "p":
                    print_points(records)
                elif command == "q":
                    break
                else:
                    print("Use r / p / q")
            print("Step teaching finished. Saved:", save_path)
            return

        # Batch mode: all leader points have been captured before Lumos replay starts.
        targets = [map_joint_to_lumos(record["leader_joints_deg"], leader_reference,
                                      lumos_reference, args.max_offset_deg)
                   for record in records]
        check_lumos_path(lumos_reference, targets, args.max_jump_deg)
        for index, target in enumerate(targets, 1):
            records[index - 1]["lumos_target_rad"] = target
            print(f"Lumos target {index} (rad):", [round(value, 4) for value in target])
        data["records"] = records
        save_json(save_path, data)
        print("Saved complete teaching path:", save_path)

        if not args.execute:
            print("PREVIEW ONLY. Re-run with --execute to replay the batch.")
            return

        current = validate_six(arm.get_joint_positions(), "Lumos joint feedback")
        if any(abs(a - b) > 0.08 for a, b in zip(current, lumos_reference)):
            raise RuntimeError("Lumos moved away from its captured reference pose; aborting replay")
        full_path = [lumos_reference] + targets
        gripper_path = None
        if args.gripper:
            gripper_path = [lumos_gripper_reference] + [record["gripper_position"] for record in records]
        print("WARNING: joint mapping is an approximate relative mapping, not a calibrated kinematic conversion.")
        if input("Inspect every target and confirm the physical path is collision-free; type REPLAY: ").strip() != "REPLAY":
            print("Replay cancelled.")
            return

        if gripper_path is None:
            execute_batch(arm, full_path, None, args)
        else:
            execute_batch(arm, full_path, gripper_path, args)

        actual = validate_six(arm.get_joint_positions(), "Lumos joint feedback")
        final_error = max(abs(a - b) for a, b in zip(actual, targets[-1]))
        print(f"Batch replay complete; final max joint error={final_error:.3f} rad")
        if final_error > 0.15:
            raise RuntimeError("Final waypoint was not reached within 0.15 rad")
        if args.gripper:
            actual_gripper = float(arm.get_gripper_position())
            print(f"Final gripper feedback={actual_gripper:.3f}; target={gripper_path[-1]:.3f}")

    except KeyboardInterrupt:
        print("\nStopped by keyboard. An already-running Lumos trajectory may continue.")
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise
    finally:
        if arm is not None:
            arm.cleanup()
        if uart is not None and uart.is_open:
            uart.close()


if __name__ == "__main__":
    main()
