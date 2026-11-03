#!/usr/bin/env python3
"""Hand-guide Lumos, capture joint waypoints, then optionally replay them."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import queue
import sys
import threading
import time

DEFAULT_CAN = "can0"
SAMPLE_HZ = 20.0
MAX_RECORD_SEC = 120.0
REPLAY_SPEED_PERCENT = 0.1


def find_sdk_root() -> Path | None:
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


def read_joints(arm) -> list[float]:
    values = [float(value) for value in arm.get_joint_positions()]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise RuntimeError("Invalid Lumos joint feedback; expected six finite values")
    return values


def load_recording(path: Path) -> tuple[list[list[float]], float]:
    data = json.loads(path.read_text(encoding="utf-8"))
    raw_waypoints = data.get("waypoints_rad")
    if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
        raise ValueError("Recording must contain at least two joint samples")
    waypoints = []
    for index, point in enumerate(raw_waypoints, 1):
        values = [float(value) for value in point]
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError(f"Invalid waypoint {index}: expected six finite joint values")
        waypoints.append(values)
    duration = float(data.get("recording_duration_sec"))
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Recording has an invalid recording_duration_sec")
    return waypoints, duration


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="Output subdirectory name under ./output")
    args = parser.parse_args()
    if (Path(args.name).name != args.name or args.name in (".", "..")
            or not args.name.strip()):
        parser.error("name must be a single folder name, not a path")
    output_dir = (Path.cwd() / "output" / args.name).resolve()
    recording_path = output_dir / "lumos_teach.json"
    mode = input("Type r to record a new path, or p to replay the saved path: ").strip().lower()
    if mode not in ("r", "p"):
        print("Cancelled; enter r or p.")
        return 0
    if mode == "p" and not recording_path.is_file():
        print(f"Recording not found: {recording_path}", file=sys.stderr)
        return 1

    sdk_root = find_sdk_root()
    try:
        from startouchclass import SingleArm
    except ImportError as exc:
        print(f"Cannot import Lumos Python SDK: {exc}", file=sys.stderr)
        print("Install startouchclass and its dependencies in the active Python environment.",
              file=sys.stderr)
        return 2

    arm = None
    try:
        if sdk_root is not None:
            print(f"Lumos SDK config: {sdk_root / 'src/config/robot_kinematics.yaml'}")
        else:
            print("SDK config root not found in standard locations; trying current environment.")
        print("Connecting to Lumos; initialization may enable motors. Keep the E-stop accessible.")
        with sdk_initialization_directory(sdk_root):
            arm = SingleArm(can_interface_=DEFAULT_CAN, gripper=False, enable_fd_=False)
        time.sleep(1.0)

        print("Moving Lumos to the SDK home pose before recording.")
        arm.go_home()
        time.sleep(3.3)

        if mode == "p":
            waypoints, recording_duration = load_recording(recording_path)
            current = read_joints(arm)
            start_error = max(abs(math.degrees(a - b)) for a, b in zip(current, waypoints[0]))
            print(f"Loaded {len(waypoints)} samples from {recording_path}")
            print(f"Recorded duration: {recording_duration:.2f} s; start-pose difference after home: "
                  f"{start_error:.1f} deg maximum joint difference")
            print("First waypoint (rad):", [round(value, 4) for value in waypoints[0]])
            print("Last waypoint  (rad):", [round(value, 4) for value in waypoints[-1]])
            if input("Check the full path and clear workspace; type y to replay: ").strip().lower() != "y":
                print("Replay cancelled.")
                return 0
            planned_duration = arm.set_joint_waypoints(
                waypoints, speed_percent=REPLAY_SPEED_PERCENT
            )
            if planned_duration is not None:
                print(f"SDK planned duration at {REPLAY_SPEED_PERCENT:.0%} speed: "
                      f"{float(planned_duration):.2f} seconds")
            print("Replay command sent.")
            return 0

        print("Enabling SDK gravity compensation for hand-guiding.")
        arm.gravity_compensation()
        time.sleep(0.3)
        print(f"Continuous recording at {SAMPLE_HZ:g} Hz.")
        print("Hand-guide the arm now. Press Enter to stop recording.")
        stop_recording = threading.Event()
        input_result: queue.Queue[BaseException | None] = queue.Queue()

        def wait_for_stop():
            try:
                input("Press Enter when the hand-guided path is complete: ")
                input_result.put(None)
            except BaseException as exc:
                input_result.put(exc)
            finally:
                stop_recording.set()

        threading.Thread(target=wait_for_stop, daemon=True).start()
        waypoints = []
        period = 1.0 / SAMPLE_HZ
        recording_started = time.monotonic()
        next_sample = recording_started
        last_report = recording_started
        while not stop_recording.is_set():
            now = time.monotonic()
            if now - recording_started >= MAX_RECORD_SEC:
                print("Maximum recording duration reached.")
                stop_recording.set()
                break
            try:
                waypoints.append(read_joints(arm))
            except Exception:
                stop_recording.set()
                raise
            next_sample += period
            delay = next_sample - time.monotonic()
            if delay > 0:
                stop_recording.wait(delay)
            else:
                next_sample = time.monotonic()
            if time.monotonic() - last_report >= 1.0:
                print(f"Recorded {len(waypoints)} samples ({time.monotonic() - recording_started:.1f}s)")
                last_report = time.monotonic()

        if not input_result.empty():
            input_error = input_result.get_nowait()
            if input_error is not None:
                raise input_error
        recording_duration = max(time.monotonic() - recording_started, 0.0)
        if waypoints:
            final = read_joints(arm)
            if any(abs(a - b) > 1e-6 for a, b in zip(final, waypoints[-1])):
                waypoints.append(final)
        print(f"Recording stopped: {len(waypoints)} samples over {recording_duration:.2f}s.")

        if len(waypoints) < 2 or recording_duration < 0.1:
            print("Recording was too short; nothing to replay.")
            return 0

        output_dir.mkdir(parents=True, exist_ok=True)
        recording_path.write_text(json.dumps({
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "can_interface": DEFAULT_CAN,
            "sample_hz": SAMPLE_HZ,
            "recording_duration_sec": recording_duration,
            "waypoints_rad": waypoints,
        }, indent=2), encoding="utf-8")
        print(f"Saved {len(waypoints)} samples to {recording_path}")

        print(f"Replay path: {len(waypoints)} samples; SDK speed limit setting "
              f"{REPLAY_SPEED_PERCENT:.0%} (planned duration may differ from recording).")
        if input("Recording saved. Check the path and clear workspace; type y to replay now, or press Enter to finish: ").strip().lower() != "y":
            print(f"Replay later by running `python {Path(__file__).name} {args.name}` and choosing p.")
            return 0
        returned_duration = arm.set_joint_waypoints(
            waypoints, speed_percent=REPLAY_SPEED_PERCENT
        )
        if returned_duration is not None:
            print(f"SDK planned duration at {REPLAY_SPEED_PERCENT:.0%} speed: "
                  f"{float(returned_duration):.2f} seconds")
        print("Replay command sent.")
        return 0
    except KeyboardInterrupt:
        print("\nCtrl+C received; commanding Lumos to the SDK home pose.")
        if arm is not None:
            try:
                arm.go_home()
                time.sleep(3.3)
                print("Home command completed.")
            except Exception as exc:
                print(f"Could not command Lumos home: {exc}", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if arm is not None:
            try:
                arm.cleanup()
            except Exception as exc:
                print(f"Lumos cleanup warning: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
