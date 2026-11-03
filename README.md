# StarArm 102 to Lumos

This repository contains standalone scripts for testing and teleoperating a Lumos arm, including direct hand-guided teaching without a StarArm leader. They use the vendor SDKs directly.

| Script | Purpose |
| --- | --- |
| `01_raw_teleop.py` | Continuous leader-to-follower teleoperation with calibration, filtering, speed limiting, and optional gripper control. |
| `02_lumos_joint_test.py` | Standalone Lumos home and joint-offset test. |
| `03_keyboard_teach.py` | Keyboard-triggered point teaching or record-then-replay teaching, with optional gripper control. |
| `04_lumos_teach.py` | Hand-guide Lumos directly, record joint waypoints, and optionally replay them without a StarArm leader. |

## Requirements and SDK discovery

Use the same Python environment for all commands. It must have `pyserial`, `fashionstar_uart_sdk`, `startouchclass`, and the Lumos native SDK dependencies installed. The Python modules must be importable by the active `python`; `--sdk-python-path` is not used.

Scripts 01 and 03 accept an optional `--sdk-root`. Script 02 automatically checks for `src/config/robot_kinematics.yaml`. The default search locations are:

1. `<script directory>/.dependencies/startouch_sdk`
2. `<script directory>/startouch_sdk`
3. The current working directory
4. `<current working directory>/startouch_sdk`

This repository does not include `.dependencies` or the Lumos SDK source/configuration. If the SDK config is outside those locations, pass `--sdk-root` to scripts 01 or 03, or place/run the SDK so its configuration is discoverable for script 02. Finding the config does not install the Python package; `startouchclass` must still import successfully from the active environment. All scripts default to CAN interface `can0` and the leader serial port `/dev/ttyUSB0` where applicable.

## 01: Continuous teleoperation

Calibrate once, inspect a preview, then execute:

```bash
python 01_raw_teleop.py --calibrate
python 01_raw_teleop.py
python 01_raw_teleop.py --execute
```

Calibration records the leader reference pose and observed joint ranges, along with the current Lumos joint pose. The calibration file defaults to `~/.config/stararm102_lumos/calibration.json`. Runtime checks the leader port, CAN interface, joint map, and gripper setting against the calibration.

Default command-line values and behavior:

- `--leader-port /dev/ttyUSB0`; `--can can0`
- `--joint-map 0,1,2,3,4,5`
- Preview by default; `--execute` sends continuous joint targets
- HD button board enabled by default at ID 7; use `--button-disable` if absent or `--button-id ID` to select another ID
- Gripper disabled by default; use `--gripper` for both calibration and runtime, and `--invert-gripper` if its direction is reversed
- Use `--diagnostics-file timing.csv` to save loop timing and target data

The six joint direction signs, gain, speed cap, filter, and control rate are constants near the top of the script. The loop sends `set_joint_raw(target, velocities)` while executing; it is designed for continuously updated targets.

## 02: Lumos joint test

`--action` is required. Without it, argparse prints an error and exits:

```bash
python 02_lumos_joint_test.py --action joint
python 02_lumos_joint_test.py --action home
```

Options and defaults:

- `--action {home,joint}`: required. `home` targets all-zero joint angles; `joint` offsets the current measured pose.
- `--can can0`: CAN interface
- `--delta-deg 5 0 0 0 0 0`: six degree offsets for `joint`
- `--time-sec 3.0`: duration for `joint`; `go_home()` uses the SDK's fixed 3-second move
- `--max-change-deg 60.0`: maximum allowed change guard for either action; choose a limit appropriate to the real setup
- Preview by default; `--execute` enables motion, followed by typing `r` at the confirmation prompt

Examples:

```bash
python 02_lumos_joint_test.py --action joint --delta-deg 2 0 0 0 0 0
python 02_lumos_joint_test.py --action joint --delta-deg 2 0 0 0 0 0 --execute
python 02_lumos_joint_test.py --action home --execute
```

The default joint action is a 5-degree change at J1. The home action can be a much larger move from the current pose; the change guard may reject it unless `--max-change-deg` is raised deliberately. Review the printed current pose, goal, and change before enabling motion.

## 03: Keyboard teaching

Basic preview and execution commands:

```bash
python 03_keyboard_teach.py --button-disable
python 03_keyboard_teach.py --button-disable --execute
python 03_keyboard_teach.py --mode batch --button-disable --execute
```

The default leader port is `/dev/ttyUSB0`, CAN is `can0`, mode is `step`, save file is `stararm_lumos_teach.json` in the launch directory, and the HD button board is enabled at ID 7. Use `--button-disable` when there is no board. `--sdk-root PATH` can identify a Lumos config root if it is outside the standard search locations.

### Step mode (default)

The script captures a leader reference pose, connects Lumos, then waits for commands at `step>`:

- Move the leader to a point and enter `r` to record and propose its follower target.
- Enter `p` to list recorded points, or `q` to finish.
- Without `--execute`, the script only prints targets. With `--execute`, it asks for `r` once at the safety confirmation prompt before teaching begins. Later, at `step>`, enter `r` to record and command Lumos to move to that target.

Each step uses a single-point `set_joint(target, tf=...)` command. The script checks the leader offset against the reference and the jump from the previous leader point. Defaults are 3 seconds per segment, 30 degrees maximum offset from reference, and 25 degrees maximum change between adjacent recorded points. A rejected point (for example, an offset of 48.8 degrees) is not sent to Lumos. Return the leader near its reference pose or adjust the constants near the top of the script only after checking the physical joint limits.

### Batch mode

Run with `--mode batch`. At `teach>` enter `r` to record a point, `d` to delete the last point, `p` to list points, and `q` to finish recording. The script then connects Lumos and builds a path from the measured Lumos reference pose through the recorded targets.

In preview mode, it prints and saves the path without moving. With `--execute`, it asks for `r` at the safety confirmation prompt before teaching begins, then asks for `y` after displaying the complete target path to confirm replay. Batch execution uses `set_joint_waypoints()` for joint path replay. This mode records several points first and sends them as a trajectory, while step mode sends each accepted point as its own command. The script currently collects a new path each run; it does not load an earlier JSON file for playback.

### Gripper and shared settings

Add `--gripper` to calibrate the leader gripper and control the Lumos gripper; use it consistently for the run. Add `--invert-gripper` if opening/closing is reversed. Batch mode uses the native joint-waypoint plus gripper API when available; otherwise it interpolates gripper commands from a Python thread, so gripper timing is approximate.

Both modes use the same direction signs and relative mapping. `FOLLOW_GAIN` is 1.0. The HD button is read while waiting for keyboard input and controls leader holding torque; it does not stop an already-started Lumos trajectory.

## 04: Hand-guided Lumos teaching

This script does not connect to StarArm. It initializes Lumos and first calls `go_home()`. It then calls the SDK wrapper's `gravity_compensation()` method for hand-guiding and continuously samples all six joint positions while you move the arm. Press Enter once to stop recording; there is no per-waypoint keypress. The recorded trajectory is sent to `set_joint_waypoints()` for replay. Pressing Ctrl+C commands `go_home()` before SDK cleanup.

```bash
python 04_lumos_teach.py task1
```

The command-line argument is only the project name. It creates `./output/<name>/lumos_teach.json`; for example, `task1` writes to `./output/task1/lumos_teach.json`. At startup, enter `r` to record a new path or `p` to replay the saved path. Recording uses CAN `can0`, automatic sampling at 20 Hz, and a 120-second maximum duration. Press Enter to stop recording. After saving, review the path and type `y` to replay immediately, or press Enter to finish. To replay later, run the same command again and choose `p`; inspect the printed path and start-pose difference, then type `y` to replay. Playback uses `set_joint_waypoints()` with the SDK's speed percentage set to 10%, allowing the SDK to plan a duration that satisfies its configured motion limits. Replay duration can therefore be longer than the original recording.

Lumos initialization may enable motors. Startup homing and the Ctrl+C home command move the arm to the SDK's zero joint pose and may cross the workspace. Ensure that path is clear before starting and keep the E-stop accessible. Hand-guiding relies on the installed SDK/controller's gravity-compensation behavior.

## Motion API calls used in this repository

- `01_raw_teleop.py`: `set_joint_raw()` for streamed joint targets.
- `02_lumos_joint_test.py`: `go_home()` or `set_joint()` depending on `--action`.
- `03_keyboard_teach.py` step mode: `set_joint()`.
- `03_keyboard_teach.py` batch mode: `set_joint_waypoints()`; with gripper, it may call `move_joint_waypoints_with_gripper()` when the installed SDK provides it.
- `04_lumos_teach.py`: `gravity_compensation()` for hand-guiding, then `set_joint_waypoints()` for replay.

The scripts do not call Cartesian `movel()` or `movep()`. The SDK wrapper contains `MotionProgram` methods for `movej()`, `movel()`, and `movep()`, but these are not used by the teleoperation/test scripts.

## Safety notes

- `--execute` is required for commanded Lumos movement in scripts 01–03. Script 04 uses interactive `y` confirmation for replay. Lumos initialization can still enable motors in preview/recording mode.
- In 02, the action must also pass the change guard and the `r` confirmation prompt. In 03, step points must pass offset/jump guards; batch replay also requires the `y` prompt. In step mode, `r` at the later `step>` prompt records a point and triggers its command.
- Verify joint order, signs, limits, workspace, and collision-free paths before execution. Keep the hardware E-stop accessible.
- Ctrl+C may not cancel a trajectory already accepted by the controller.
- These scripts have not been validated on hardware as part of this repository change.
