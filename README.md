# StarArm 102 → Lumos 直接 SDK 脚本

三个脚本直接调用原厂 SDK，不需要额外的集成框架。

| 脚本 | 用途 |
| --- | --- |
| `01_raw_teleop.py` | 校准、50 Hz 连续遥操作、滤波和限速、可选夹爪 |
| `02_lumos_joint_test.py` | Lumos 回零与关节运动测试 |
| `03_keyboard_teach.py` | 键盘逐点跟随、批量示教与回放、可选夹爪 |

## SDK 环境

在同一个 Python 环境安装 `pyserial`、StarArm 的 `fashionstar_uart_sdk` 和 Lumos 的 `startouchclass` / 原生扩展。请按各 SDK 的安装说明安装，脚本直接 import，不再传 `--sdk-python-path`。

01 和 03 自动查找以下位置的 `src/config/robot_kinematics.yaml`：

1. 脚本旁 `.dependencies/startouch_sdk`
2. 脚本旁 `startouch_sdk`
3. 当前工作目录
4. 当前工作目录下 `startouch_sdk`

SDK 位于其他位置时，只需可选参数 `--sdk-root /实际路径/startouch_sdk`。该参数指定配置根目录；Python 包仍需安装到当前环境。初始化后恢复原工作目录，保存文件相对启动目录解析。

02 保留原有独立测试接口，其参数可通过 `python 02_lumos_joint_test.py --help` 查看。

## 01 连续遥操作

```bash
python 01_raw_teleop.py --calibrate
python 01_raw_teleop.py
python 01_raw_teleop.py --execute
```

先校准参考姿态和主臂活动范围，再预览核对关节方向，最后执行。夹爪需在校准和运行时均加 `--gripper`，方向相反时加 `--invert-gripper`。默认串口 `/dev/ttyUSB0`、CAN `can0`，需要时用 `--leader-port`、`--can` 修改。

默认启用 ID 7 的 HD 按钮；无按钮板时添加 `--button-disable`。预览模式也会控制主臂悬停力矩。

关节方向、跟随比例、速度、滤波和循环频率在脚本顶部修改。诊断可用 `--diagnostics-file timing.csv`。

## 03 键盘示教

```bash
python 03_keyboard_teach.py
python 03_keyboard_teach.py --mode batch
python 03_keyboard_teach.py --gripper --execute
python 03_keyboard_teach.py --mode batch --gripper --execute
```

默认 `step` 模式：主臂摆到一个点，输入 `r` 并按 Enter 记录；加 `--execute` 时 Lumos 规划运动到该点。输入 `p` 查看，`q` 结束。这是按键触发的逐点运动。

`batch` 模式：输入 `r` 记录、`d` 删除末点、`p` 查看、`q` 结束采集；随后连接 Lumos 并生成目标，执行模式回放整条路径。记录默认保存到启动目录下 `stararm_lumos_teach.json`，可通过 `--save` 修改。当前脚本每次重新采集，不从已有文件回放。

03 使用与 01 相同的 SDK 查找方式、关节方向和默认 HD 按钮行为。等待键盘输入期间也持续读取按钮。无按钮板时添加 `--button-disable`。逐点和批量运动使用 SDK 的轨迹接口，运动期间不轮询 HD 按钮；HD 按钮只控制主臂悬停，不停止 Lumos 轨迹。

示教参数集中在顶部：每段默认 3 秒、相对参考姿态最大偏移 30°、相邻记录点最大变化 25°，直接修改顶部常量即可。03 每次独立记录参考姿态，不读取 01 的校准文件。

批量夹爪优先使用原生同步接口；不支持时采用 Python 定时插值，其时间同步为近似值。

## 操作注意

- 默认预览，加 `--execute` 才发送 Lumos 运动命令；03 保留 RUN / REPLAY 确认。
- `SingleArm` 初始化可能使能电机，校准和预览也一样。
- 映射为参考姿态上的关节角增量，使用前核对关节顺序、方向、机械限位及无碰撞路径。
- 保持硬件急停可用；Ctrl+C 可能无法取消控制器中已经启动的轨迹。
- 尚未进行实机验证。
