#!/usr/bin/env python3
"""StarArm 102 → Lumos 键盘示教：逐点跟随或批量回放，支持夹爪和 HD 按钮。"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from typing import Sequence
import queue
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
# ===================== 用户可调整的示教参数 =====================
JOINT_SIGNS = (-1.0, 1.0, 1.0, 1.0, -1.0, -1.0)
FOLLOW_GAIN = 1.0
SECONDS_PER_SEGMENT = 0.5
MAX_OFFSET_DEG = 60.0
MAX_JUMP_DEG = 60.0
GRIPPER_RATE_HZ = 20.0
CONTROL_RATE_HZ = 50.0
HD_BUTTON_ENABLED = True
HD_BUTTON_ID = 7
# =============================================================


class HDButton:
    """Match python-sdk/stararm102_ro.py: 180 degrees locks joints 0..5."""

    def __init__(self, leader, button_id: int):
        self.leader = leader
        self.id = button_id
        self.locked: bool | None = None
        if not leader.ping(button_id):
            raise RuntimeError(
                f"HD 按钮板 ID {button_id} 未响应。请检查连接/ID；无按钮时加 --button-disable。"
            )

    def update(self) -> None:
        angle = float(self.leader.servos[self.id].angle_monitor)
        if not math.isfinite(angle):
            raise RuntimeError(f"HD 按钮板 ID {self.id} 返回了无效状态")
        locked = angle == 180.0
        if locked == self.locked:
            return
        # Only affect the six arm joints; never lock the gripper or button board.
        for servo_id in LEADER_JOINT_IDS:
            self.leader.stop_on_control_mode(servo_id, 0x11 if locked else 0x10, 0x00)
        self.locked = locked
        print("\nHD 主臂：悬停锁定" if locked else "\nHD 主臂：释放，可拖动")


def read_leader(leader, joint_map: Sequence[int], include_gripper: bool = False,
                hd_button: HDButton | None = None) -> list[float]:
    ids = list(joint_map)
    if include_gripper:
        ids.append(6)
    monitor_ids = ids + ([hd_button.id] if hd_button is not None else [])
    leader.send_sync_servo_monitor(monitor_ids)
    if hd_button is not None:
        hd_button.update()
    values = [float(leader.servos[servo_id].angle_monitor) for servo_id in ids]
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("主臂返回无效角度，停止示教")
    return values


def load_sdk():
    try:
        import serial
        import fashionstar_uart_sdk as uservo
        from startouchclass import SingleArm
    except ImportError as exc:
        raise RuntimeError(
            f"缺少 Python SDK：{exc}。请在同一个 Python 环境安装 pyserial、fashionstar-uart-sdk 和 Lumos startouch_sdk。"
        ) from exc
    return serial, uservo, SingleArm


def find_sdk_root(explicit: Path | None = None) -> Path | None:
    script_dir = Path(__file__).resolve().parent
    candidates = ([explicit] if explicit is not None else [
        script_dir / ".dependencies" / "startouch_sdk",
        script_dir / "startouch_sdk", Path.cwd(), Path.cwd() / "startouch_sdk",
    ])
    for candidate in candidates:
        root = candidate.expanduser().resolve()
        if (root / "src" / "config" / "robot_kinematics.yaml").is_file():
            return root
    if explicit is not None:
        raise RuntimeError(f"SDK 配置不存在：{explicit}/src/config/robot_kinematics.yaml")
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



def prompt_with_button(prompt, bus, hd_button):
    """等待键盘时持续读取 HD 按钮；串口只由主线程访问。"""
    if hd_button is None:
        return input(prompt)
    result = queue.Queue()

    def read_input():
        try:
            result.put((input(prompt), None))
        except BaseException as exc:
            result.put((None, exc))

    threading.Thread(target=read_input, daemon=True).start()
    while True:
        try:
            value, error = result.get(timeout=1.0 / CONTROL_RATE_HZ)
        except queue.Empty:
            read_leader(bus, LEADER_JOINT_IDS, hd_button=hd_button)
            continue
        if error is not None:
            raise error
        return value



def map_joint_to_lumos(leader_deg, leader_ref_deg, lumos_ref_rad, max_offset_deg):
    """Map leader joint offsets (deg) onto Lumos reference joints (rad)."""
    offsets = [JOINT_SIGNS[i] * FOLLOW_GAIN * (leader_deg[i] - leader_ref_deg[i]) for i in range(6)]
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
        print("尚无示教点。")
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
    parser.add_argument("--gripper", action="store_true",
                        help="Also teach StarArm servo 6 and control the Lumos gripper")
    parser.add_argument("--invert-gripper", action="store_true",
                        help="Invert the leader-to-Lumos gripper opening direction")
    buttons = parser.add_mutually_exclusive_group()
    buttons.add_argument("--button-enable", action="store_true", dest="button_enabled")
    buttons.add_argument("--button-disable", action="store_false", dest="button_enabled",
                         help="无 HD 按钮板时关闭按钮功能")
    parser.set_defaults(button_enabled=HD_BUTTON_ENABLED)
    parser.add_argument("--button-id", type=int, choices=range(7, 254), default=HD_BUTTON_ID, metavar="ID")
    parser.set_defaults(seconds_per_segment=SECONDS_PER_SEGMENT,
                        max_offset_deg=MAX_OFFSET_DEG, max_jump_deg=MAX_JUMP_DEG,
                        gripper_rate_hz=GRIPPER_RATE_HZ)
    parser.add_argument("--save", type=Path, default=Path("stararm_lumos_teach.json"))
    parser.add_argument("--execute", action="store_true",
                        help="Actually command Lumos; without it, only preview targets")
    args = parser.parse_args()
    args.save = args.save.expanduser().resolve()
    if len(JOINT_SIGNS) != 6 or any(sign not in (-1.0, 1.0) for sign in JOINT_SIGNS):
        parser.error("JOINT_SIGNS 必须包含六个 1.0 或 -1.0")

    parameters = (args.seconds_per_segment, args.max_offset_deg,
                  args.max_jump_deg, args.gripper_rate_hz, FOLLOW_GAIN, CONTROL_RATE_HZ)
    if (not all(math.isfinite(value) and value > 0 for value in parameters)
            or FOLLOW_GAIN > 1 or CONTROL_RATE_HZ > 100):
        parser.error("time, safety limits, and gripper rate must be positive")

    try:
        sdk_root = find_sdk_root(args.sdk_root)
        serial, uservo, SingleArm = load_sdk()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 2

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

        hd_button = HDButton(bus, args.button_id) if args.button_enabled else None
        if hd_button is not None:
            print(f"HD 按钮功能已开启（ID={hd_button.id}）；预览模式也控制主臂悬停。")
            read_leader(bus, LEADER_JOINT_IDS, hd_button=hd_button)

        def ask(prompt):
            return prompt_with_button(prompt, bus, hd_button)

        gripper_closed = None
        gripper_opened = None
        if args.gripper:
            ask("将 StarArm 夹爪完全闭合，按 Enter 记录……")
            gripper_closed = read_leader(bus, LEADER_JOINT_IDS, True, hd_button)[-1]
            ask("将 StarArm 夹爪完全打开，按 Enter 记录……")
            gripper_opened = read_leader(bus, LEADER_JOINT_IDS, True, hd_button)[-1]
            if abs(gripper_opened - gripper_closed) < 2.0:
                raise RuntimeError("StarArm gripper calibration span is less than 2 degrees")
            print(f"Leader gripper calibration: closed={gripper_closed:.1f}, open={gripper_opened:.1f} deg")

        ask("将 StarArm 摆到遥操作参考姿态，按 Enter 记录……")
        leader_reference = read_leader(bus, LEADER_JOINT_IDS, hd_button=hd_button)[:6]
        data["leader_reference_deg"] = leader_reference
        data["leader_gripper_closed_deg"] = gripper_closed
        data["leader_gripper_open_deg"] = gripper_opened
        print("Leader reference (deg):", [round(value, 2) for value in leader_reference])

        if args.mode == "batch":
            print("\n批量示教：移动主臂记录多个点，示教结束后回放。")
            print("命令：r=记录，d=删除末点，p=查看，q=结束示教（输入后按 Enter）")
            while True:
                command = ask("teach> ").strip().lower()
                if command == "r":
                    sample = read_leader(bus, LEADER_JOINT_IDS, args.gripper, hd_button)
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
                        print("没有可删除的示教点。")
                elif command == "p":
                    print_points(records)
                elif command == "q":
                    break
                else:
                    print("请输入 r / d / p / q")
        else:
            print("\n逐点示教：主臂摆到目标点后输入 r；执行模式下 Lumos 跟随到该点。")
            print("命令：r=记录并跟随，p=查看，q=结束")

        if args.mode == "batch" and not records:
            print("没有记录示教点，退出。")
            return

        if args.mode == "batch":
            print("\n主臂示教结束，请将 Lumos 摆到对应的参考姿态。")
            ask("确认工作区域和急停，按 Enter 连接 Lumos……")
        else:
            print("\n逐点示教：开始前请将 Lumos 摆到对应的参考姿态。")
            ask("确认工作区域和急停，按 Enter 连接 Lumos……")

        print("Lumos 连接时可能使能电机，预览模式也一样；请确认急停可用。")
        if sdk_root is not None:
            print(f"Lumos SDK 配置：{sdk_root / 'src/config/robot_kinematics.yaml'}")
        with sdk_initialization_directory(sdk_root):
            arm = SingleArm(can_interface_=args.can, gripper=args.gripper, enable_fd_=False)
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
            if ask("确认工作区域、限位和急停，输入 r 开始：").strip().lower() != "r":
                print("Cancelled; no Lumos motion command sent.")
                return
        else:
            print("预览模式：只显示目标，不发送 Lumos 运动命令（连接仍可能使能电机）。")

        if args.mode == "step":
            print("\n命令：r=记录当前主臂姿态并触发跟随，p=查看，q=结束")
            previous_leader = leader_reference
            expected_lumos = lumos_reference[:]
            while True:
                command = ask("step> ").strip().lower()
                if command == "r":
                    sample = read_leader(bus, LEADER_JOINT_IDS, args.gripper, hd_button)
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
                    print("请输入 r / p / q")
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
        if ask("检查全部目标并确认路径无碰撞，输入 y 回放：").strip().lower() != "y":
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
        print("\n键盘停止。Lumos 控制器中已启动的轨迹可能继续执行。")
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if arm is not None:
            try:
                arm.cleanup()
            except Exception as exc:
                print(f"Lumos cleanup warning: {exc}", file=sys.stderr)
        if uart is not None and uart.is_open:
            uart.close()


if __name__ == "__main__":
    raise SystemExit(main())
