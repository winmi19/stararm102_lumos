# StarArm 102 + Lumos: direct-SDK scripts

This project keeps the original Lumos `SingleArm` SDK and directly imports the StarArm UART SDK. There is no extra integration framework or `common.py` layer.

## Scripts

| Script | Purpose | Main Lumos API |
| --- | --- | --- |
| `01_raw_teleop.py` | 50 Hz raw joint servo teleoperation; optional gripper | `set_joint_raw`, `setGripperPosition` |
| `02_lumos_joint_test.py` | Test Lumos home and planned joint motion | `go_home`, `set_joint` |
| `03_keyboard_teach.py` | One keyboard teaching script with two replay modes and optional gripper | `set_joint`, `set_joint_waypoints`, `move_joint_waypoints_with_gripper` when supported |

## Install and SDK paths

Use the Python environment where the Lumos `startouch` native extension is installed. Install the supplied StarArm SDK source with:

```bash
python -m pip install -e /path/to/python-sdk
python -m pip install pyserial numpy
```

If Lumos is not already on the Python path, pass:

- `--sdk-root /absolute/path/to/startouch_sdk` (contains `src/config/robot_kinematics.yaml`)
- `--sdk-python-path /absolute/path/to/interface_py` (contains `startouchclass.py`)

## Keyboard teaching modes

### Mode A: step-follow (leader reaches a point, keypress triggers follower)

```bash
python 03_keyboard_teach.py --mode step --leader-port /dev/ttyUSB0 --can can0
```

Move the StarArm leader to a point, type `r` and press Enter. Lumos immediately plans to that corresponding point using `set_joint()`. Repeat for each point; type `p` to print or `q` to finish. This is waypoint-triggered following, not continuous real-time following while the leader is moving.

With gripper:

```bash
python 03_keyboard_teach.py --mode step --gripper --leader-port /dev/ttyUSB0 --can can0
```

### Mode B: batch teaching (record all leader points, then follower replays)

```bash
python 03_keyboard_teach.py --mode batch --leader-port /dev/ttyUSB0 --can can0
```

Type `r` at each leader waypoint, `d` to delete the last point, `p` to print, and `q` to finish teaching. After teaching, place Lumos at the corresponding reference pose. The script maps all points and previews them. Add `--execute` to replay the path using one `set_joint_waypoints()` request.

With gripper:

```bash
python 03_keyboard_teach.py --mode batch --gripper --leader-port /dev/ttyUSB0 --can can0
```

If the installed C++ extension exposes native `move_joint_waypoints_with_gripper`, the script uses the SDK's joint+gripper waypoint API. If not, it runs a Python-timed gripper interpolation alongside the joint waypoint request; this is approximate timing, not hardware-time synchronization.

### Execute real movement

All scripts preview by default. Add `--execute` to request real Lumos commands. The teaching script also asks for explicit `RUN` and `REPLAY` confirmations. Example:

```bash
python 03_keyboard_teach.py --mode batch --gripper --execute \
  --leader-port /dev/ttyUSB0 --can can0 \
  --sdk-root /path/to/startouch_sdk \
  --sdk-python-path /path/to/startouch_sdk/interface_py
```

If the StarArm leader is torque-held and needs to be hand-guided, add `--release-leader`. This explicitly sends the unload command to leader joints 0..5 after a second confirmation. Support the arm first; unloading torque can make it fall. The script does not automatically re-enable leader torque.

## Mapping and safety

The relative mapping is:

```text
target[i] = lumos_reference[i] + sign[i] * radians(leader[i] - leader_reference[i])
```

The signs `(-1, 1, 1, 1, -1, -1)` are inherited from the earlier teleoperation bridge and have not been verified as a calibrated conversion between two different robots. The default maximum leader offset is 30 degrees and maximum adjacent recorded step is 25 degrees. These are conservative software guards, not the robots' mechanical joint limits.

Important:

- `SingleArm(...)` may enable Lumos motors even in preview mode.
- Confirm joint order, direction, limits, reference poses, gripper opening direction, and collision-free workspace before execution.
- Keep the physical emergency stop accessible. Software checks are not an emergency stop.
- `Ctrl+C` may not cancel an already-running trajectory in the Lumos controller.
- The step mode triggers follower movement after each `r` keypress; it does not mirror the leader continuously during manual movement.
- For batch gripper playback, the Python fallback timing is approximate if native synchronized joint+gripper playback is unavailable.
- No real hardware execution has been verified as part of generating these scripts.
