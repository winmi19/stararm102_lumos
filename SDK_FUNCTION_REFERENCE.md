# Lumos Python SDK 函数开发参考

本文对应本仓库新增的 [`startouchclass.py`](startouchclass.py)，版本常量 **0.1.9**。按实际 Python 实现整理全部函数、构造器、属性和内部辅助方法；同级 `startouch_sdk` 的 API/夹爪说明仅用于补充底层约定。不是已安装原生扩展的运行验证报告。

## 阅读与调用约定

- 依赖：`numpy`、原生扩展 `startouch`；`startouchclass.py` 是包装层，复制该文件不能替代原生扩展和机器人配置。
- 关节：六个值按 J1…J6；角度 rad、角速度 rad/s、力矩 Nm。StarArm 的角度反馈为度时先用 `math.radians()` 转换。
- 位姿：`[x,y,z,roll,pitch,yaw]`，位置 m、角度 rad；坐标系、TCP 和安装方向由底层配置定义。
- 四元数：常用接口均为 **wxyz**；只有 `quaternion_to_euler_xyzw()` 使用 **xyzw**。
- 夹爪：归一化开度 0=闭合、1=最大打开；物理距离 m；TypeNex 总张开角 rad；`effort_nm` 是电机输出轴力矩，不能直接视为指尖力 N。
- 带 `time_sec` / `speed_percent` 的接口：通常二选一；`time_sec` 是整条路径总时长。都不提供时默认 `speed_percent=0.1`。时间和速度的边界处理详见 `_normalize_time_speed()`。
- 多个单目标接口无条件返回 `True`，仅说明 Python 调用未抛异常，**不能证明底层接受或机械臂已经到位**。通过反馈判断实际执行情况。
- waypoint / 笛卡尔路径 / MotionProgram 的阻塞语义有 SDK 文档依据；其余仅薄封装的方法，不在本文推断是否阻塞。

## 初始化与本仓库路径说明

```python
from startouchclass import SingleArm, MotionProgram
import numpy as np

# 连接真实控制器可能使能电机。示例变量应由实际校准/示教结果提供。
arm = SingleArm(can_interface_="can0", gripper=False, enable_fd_=False)
try:
    q = arm.get_joint_positions()
    print(q)
finally:
    arm.cleanup()
```

当前文件的 `SingleArm.__init__()` 使用 `__file__` 的父目录的上一级查找 `param_csv_gripper/permutationMatrix.csv`、`pi_b.csv`、`pi_fr.csv`。由于该文件现在位于仓库根目录，实际传给底层的路径为：

```text
stararm102_lumos/../param_csv_gripper/permutationMatrix.csv
stararm102_lumos/../param_csv_gripper/pi_b.csv
stararm102_lumos/../param_csv_gripper/pi_fr.csv
```

本仓库当前没有这些 CSV，也没有原生扩展或完整配置目录。底层是否要求这些文件、缺失时是否允许降级需核对实际安装包。01/03 的 SDK 配置目录查找与这些 CSV 路径是两套逻辑，切换工作目录不会改变基于 `__file__` 计算的 CSV 路径。从本仓库运行脚本时，新增的同名文件通常会优先于安装目录里的 `startouchclass` 被导入，可通过 `print(startouchclass.__file__)` 核对。

## 接口选型

| 场景 | 建议接口 |
| --- | --- |
| 周期主从遥操作 | `set_joint_raw()`，应用层完成滤波、限速和有效性检查 |
| 单点示教跟随 | `set_joint()`，用位置反馈确认到位 |
| 多点关节回放 | `set_joint_waypoints()` |
| 多点关节与夹爪回放 | `move_joint_waypoints_with_gripper()`，先确认原生同步接口存在 |
| 笛卡尔曲线路径 | `move_p()` / `move_p_with_gripper()` |
| 笛卡尔直线路径 | `move_l()` |
| 滚动更新动作块 | `update_joint_waypoint_chunk*()`，需匹配原生扩展 |
| 位姿转关节角 | `solve_ik()` |
| 顺序任务及等待 | `MotionProgram` + `run_motion_program()` |
| 状态采集 | `get_joint_*()`、`get_ee_pose_*()`、`get_gripper_*()` |

## 全函数索引

| 函数/属性 | 作用 |
| --- | --- |
| `quaternion_to_euler_wxyz` | 把 wxyz 四元数转换为 roll/pitch/yaw。 |
| `quaternion_to_euler_xyzw` | 把 xyzw 四元数转换为 roll/pitch/yaw。 |
| `euler_to_quaternion` | 把 roll/pitch/yaw 转为四元数。 |
| `MotionProgram.__init__` | 创建空的运动程序，初始化内部步骤列表。 |
| `MotionProgram._normalize_time_speed` | 内部静态方法：统一时间模式与速度模式参数。 |
| `MotionProgram._as_2d_points` | 内部静态方法：把单点或二维路点转换为二维浮点列表。 |
| `MotionProgram.movej` | 向程序添加关节路点步骤；连续 movej 可由底层合并。 |
| `MotionProgram.movep` | 向程序添加带拐角过渡的笛卡尔路径步骤。 |
| `MotionProgram.movel` | 向程序添加笛卡尔直线路径步骤。 |
| `MotionProgram.sleep` | 向程序添加等待步骤，形成明确的步骤间断点。 |
| `MotionProgram.items` | 读取内部 MotionProgramItem 步骤列表（属性，不是方法）。 |
| `SingleArm.__init__` | 创建单臂控制器，向原生 startouch.ArmController 传入 CAN、夹爪和动力学文件参数。 |
| `SingleArm._parse_vla_frames` | 内部静态方法：解析笛卡尔位姿与夹爪的组合帧。 |
| `SingleArm._start_gripper_position_sync` | 内部方法：启动后台线程按时间下发夹爪开度。 |
| `SingleArm.go_home` | 向六个关节发送全零角度的 3 秒规划目标。 |
| `SingleArm.gravity_compensation` | 调用底层重力补偿功能，供拖动示教等模式使用。 |
| `SingleArm.set_joint` | 设置单个关节目标，使用底层带时间规划接口。 |
| `SingleArm.set_joint_raw` | 透传关节目标位置和速度，用于外部周期伺服/遥操作。 |
| `SingleArm.set_joint_waypoints` | 执行整段关节路点轨迹，推荐用于回放。 |
| `SingleArm.move_joint_waypoints` | set_joint_waypoints 的兼容别名。 |
| `SingleArm.move_joint_waypoints_with_gripper` | 执行关节路点及对应夹爪开度。 |
| `SingleArm.update_joint_waypoint_chunk` | 向底层提交新的关节轨迹块，用于滚动刷新轨迹。 |
| `SingleArm.update_joint_waypoint_chunk_with_gripper` | 提交包含关节和夹爪的新轨迹块。 |
| `SingleArm.plan_joint_waypoints_with_gripper` | 向底层请求关节和夹爪轨迹的预规划结果。 |
| `SingleArm.move_p` | 执行多点笛卡尔路径，支持拐角过渡；底层用 IK 转成关节路径。 |
| `SingleArm.move_p_with_gripper` | 执行笛卡尔组合帧，联动末端路径和夹爪。 |
| `SingleArm.get_last_waypoint_command_samples` | 获取底层最近一次 waypoint 的命令采样结果，便于诊断/分析规划命令。 |
| `SingleArm.move_l` | 执行笛卡尔直线路径。 |
| `SingleArm.run_motion_program` | 把 MotionProgram 或原生步骤列表交给底层执行。 |
| `SingleArm.set_end_effector_pose_euler` | 设置单个末端位置和欧拉角目标，调用带 tf 的底层位姿接口。 |
| `SingleArm.set_end_effector_pose_euler_raw` | 透传单个末端位置和欧拉角目标，用于伺服更新。 |
| `SingleArm.set_end_effector_pose_quat` | 将 wxyz 四元数转换为欧拉角后，调用带时间的末端位姿接口。 |
| `SingleArm.set_end_effector_pose_quat_raw` | 将 wxyz 四元数转为欧拉角，透传末端目标。 |
| `SingleArm.get_joint_positions` | 读取六个关节的实际位置。 |
| `SingleArm.get_joint_velocities` | 读取六个关节的实际速度。 |
| `SingleArm.get_joint_torques` | 读取六个关节的力矩反馈。 |
| `SingleArm.get_ee_pose_quat` | 读取末端位置和欧拉角，再将姿态转成 wxyz 四元数。 |
| `SingleArm.get_ee_pose_euler` | 读取末端位置和 roll/pitch/yaw。 |
| `SingleArm.solve_ik` | 求末端目标对应的关节角，不通过此方法直接发送运动目标。 |
| `SingleArm.openGripper` | 请求夹爪完全打开。 |
| `SingleArm.closeGripper` | 请求夹爪闭合。 |
| `SingleArm.setGripperPosition` | 设置归一化夹爪开度。 |
| `SingleArm.setGripperDistance` | 按物理开合距离控制夹爪，可选位置控制增益。 |
| `SingleArm.setGripperDistance_raw` | 用显式 kp/kd 调用 setGripperDistance 的兼容包装。 |
| `SingleArm.setGripperDistanceEffort` | 设置物理夹爪开度及电机输出轴目标/限制力矩。 |
| `SingleArm.setGripperPositionEffort` | 设置归一化开度及电机目标/限制力矩。 |
| `SingleArm.setGripperAngle` | 设置 TypeNex 从机械闭合点开始的总张开角。 |
| `SingleArm.setGripperAngleEffort` | 设置 TypeNex 总张开角及电机目标/限制力矩。 |
| `SingleArm.setGripperAngleInterpolated` | 从当前 TypeNex 开度角出发，按绝对时间调度分步线性插值到目标角。 |
| `SingleArm.get_gripper_position` | 读取归一化夹爪开度。 |
| `SingleArm.get_gripper_distance` | 读取物理夹爪开合距离。 |
| `SingleArm.get_gripper_angle` | 读取 TypeNex 从机械闭合位置起算的总张开角。 |
| `SingleArm.get_gripper_effort` | 读取夹爪电机输出轴力矩反馈。 |
| `SingleArm.get_gripper_state` | 获取完整原生夹爪状态对象。 |
| `SingleArm.cleanup` | 调用底层清理接口，释放控制器资源。 |

## 模块级姿态转换函数

### `quaternion_to_euler_wxyz`

源码：[第 16 行](startouchclass.py#L16)。

```python
quaternion_to_euler_wxyz(quat: np.ndarray) -> np.ndarray
```

**作用：** 把 wxyz 四元数转换为 roll/pitch/yaw。

| 参数 | 含义与单位 |
| --- | --- |
| `quat` | 4 维四元数；通常 [w,x,y,z]，xyzw 转换函数例外。 |

**返回：** numpy 数组，形状 (3,)，单位 rad。

**注意：** quat 必须按 [w,x,y,z] 排列；函数没有归一化输入，应传单位四元数。

**用法：**

```python
quaternion_to_euler_wxyz(np.array([1., 0., 0., 0.]))
```

### `quaternion_to_euler_xyzw`

源码：[第 45 行](startouchclass.py#L45)。

```python
quaternion_to_euler_xyzw(q: np.ndarray) -> np.ndarray
```

**作用：** 把 xyzw 四元数转换为 roll/pitch/yaw。

| 参数 | 含义与单位 |
| --- | --- |
| `q` | 4 维 [x,y,z,w] 四元数（仅 quaternion_to_euler_xyzw）。 |

**返回：** numpy 数组，形状 (3,)，单位 rad。

**注意：** 实际实现只接收 [x,y,z,w]，不会自动识别 wxyz；原注释写“或 [w,x,y,z]”不符合实现。长度不是 4 时抛 ValueError。

**用法：**

```python
quaternion_to_euler_xyzw(np.array([0., 0., 0., 1.]))
```

### `euler_to_quaternion`

源码：[第 78 行](startouchclass.py#L78)。

```python
euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray
```

**作用：** 把 roll/pitch/yaw 转为四元数。

| 参数 | 含义与单位 |
| --- | --- |
| `roll` | 绕 X 的角，rad。 |
| `pitch` | 绕 Y 的角，rad。 |
| `yaw` | 绕 Z 的角，rad。 |

**返回：** numpy 数组，形状 (4,)，顺序 [w,x,y,z]。

**注意：** roll、pitch、yaw 均为 rad，返回格式与大多数本文件位姿接口一致。

**用法：**

```python
euler_to_quaternion(0.0, 0.0, 0.0)
```

## MotionProgram：结构化运动程序

### `MotionProgram.__init__`

源码：[第 110 行](startouchclass.py#L110)。

```python
__init__(self)
```

**作用：** 创建空的运动程序，初始化内部步骤列表。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 构造 MotionProgram 对象。

**注意：** 仅构建 Python 程序对象，不连接机械臂、不执行运动。

**用法：**

```python
program = MotionProgram()
```

### `MotionProgram._normalize_time_speed`

源码：[第 114 行](startouchclass.py#L114)。

```python
_normalize_time_speed(time_sec: Optional[float], speed_percent: Optional[float]) -> Tuple[float, float]
```

**作用：** 内部静态方法：统一时间模式与速度模式参数。

| 参数 | 含义与单位 |
| --- | --- |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |

**返回：** (time_value, speed_value)，两个 float。

**注意：** time_sec 与 speed_percent 同为正数时报错；时间负数、正速度超过 1 时报错；两者均未提供正数时使用速度 0.1。实现未完整拒绝 NaN/Inf，调用方应先检查有限值。

**用法：**

```python
MotionProgram._normalize_time_speed(None, 0.1)
```

### `MotionProgram._as_2d_points`

源码：[第 130 行](startouchclass.py#L130)。

```python
_as_2d_points(values, width: int, name: str) -> List[List[float]]
```

**作用：** 内部静态方法：把单点或二维路点转换为二维浮点列表。

| 参数 | 含义与单位 |
| --- | --- |
| `values` | 可转换为浮点 numpy 数组的一维点或二维点列表。 |
| `width` | 单点应有的维数，例如 6。 |
| `name` | 维度校验错误中的对象名称字符串。 |

**返回：** list[list[float]]。

**注意：** values 支持一维 width 个数或二维 N×width；width 为每点维数；name 用于错误信息。检查形状但不检查 NaN/Inf 或所有空数组情况。

**用法：**

```python
MotionProgram._as_2d_points([0.0]*6, 6, "waypoint")
```

### `MotionProgram.movej`

源码：[第 143 行](startouchclass.py#L143)。

```python
movej(self, waypoints: Union[List[List[float]], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None) -> 'MotionProgram'
```

**作用：** 向程序添加关节路点步骤；连续 movej 可由底层合并。

| 参数 | 含义与单位 |
| --- | --- |
| `waypoints` | 关节路点，(N,6)，rad；允许单个 6 维点。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |

**返回：** self，可链式调用。

**注意：** waypoints 为 N×6 rad；单个 6 维点也接受。这里只追加步骤，调用 run_motion_program 才执行。时间/速度遵循公共规则。

**用法：**

```python
program.movej([[0.0]*6], time_sec=3.0)
```

### `MotionProgram.movep`

源码：[第 158 行](startouchclass.py#L158)。

```python
movep(self, poses: Union[List[List[float]], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None, blend_radius_m: float=0.002) -> 'MotionProgram'
```

**作用：** 向程序添加带拐角过渡的笛卡尔路径步骤。

| 参数 | 含义与单位 |
| --- | --- |
| `poses` | 笛卡尔路点，(N,6)，[x,y,z,roll,pitch,yaw]，m/rad。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |
| `blend_radius_m` | 笛卡尔拐角过渡半径，m。 |

**返回：** self，可链式调用。

**注意：** poses 为 N×6 的 [x,y,z,roll,pitch,yaw]；blend_radius_m 默认 0.002 m。此构建接口没有 position_tolerance_m / orientation_tolerance_rad 参数。

**用法：**

```python
program.movep(poses, time_sec=3.0, blend_radius_m=0.002)
```

### `MotionProgram.movel`

源码：[第 175 行](startouchclass.py#L175)。

```python
movel(self, poses: Union[List[List[float]], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None, blend_radius_m: float=0.0) -> 'MotionProgram'
```

**作用：** 向程序添加笛卡尔直线路径步骤。

| 参数 | 含义与单位 |
| --- | --- |
| `poses` | 笛卡尔路点，(N,6)，[x,y,z,roll,pitch,yaw]，m/rad。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |
| `blend_radius_m` | 笛卡尔拐角过渡半径，m。 |

**返回：** self，可链式调用。

**注意：** poses 格式同 movep；blend_radius_m 默认 0 m。这里只记录步骤，路径可达性由底层执行/规划检查。

**用法：**

```python
program.movel(poses, time_sec=3.0)
```

### `MotionProgram.sleep`

源码：[第 192 行](startouchclass.py#L192)。

```python
sleep(self, sleep_sec: float) -> 'MotionProgram'
```

**作用：** 向程序添加等待步骤，形成明确的步骤间断点。

| 参数 | 含义与单位 |
| --- | --- |
| `sleep_sec` | 程序内等待时长，s。 |

**返回：** self，可链式调用。

**注意：** sleep_sec 单位秒；构建时不立即等待。本包装层只做 float 转换，没有校验负数或非有限值。

**用法：**

```python
program.sleep(0.5)
```

### `MotionProgram.items`

源码：[第 200 行](startouchclass.py#L200)。

```python
items(self)
```

**作用：** 读取内部 MotionProgramItem 步骤列表（属性，不是方法）。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 内部列表本身。

**注意：** 不要写 program.items()。返回的是原列表而非拷贝，修改会影响程序内容。

**用法：**

```python
items = program.items
```

## SingleArm：单臂控制器

### `SingleArm.__init__`

源码：[第 216 行](startouchclass.py#L216)。

```python
__init__(self, can_interface_='can0', gripper=True, enable_fd_=False, dry_run: bool=False)
```

**作用：** 创建单臂控制器，向原生 startouch.ArmController 传入 CAN、夹爪和动力学文件参数。

| 参数 | 含义与单位 |
| --- | --- |
| `can_interface_` | SocketCAN 接口名，默认 can0。 |
| `gripper` | 是否启用夹爪，默认 True。 |
| `enable_fd_` | 是否使用 CAN FD，默认 False。 |
| `dry_run` | 原样传给原生控制器的干跑开关，默认 False；具体行为由原生库决定。 |

**返回：** SingleArm 对象。

**注意：** can_interface_ 默认 can0；gripper 默认 True；enable_fd_ 默认 False；dry_run 默认 False，原样交给底层。初始化可能使能电机。dry_run 的仿真/通信边界需核对原生库，不等于实机只读监视模式。

**用法：**

```python
arm = SingleArm(can_interface_="can0", gripper=False, enable_fd_=False)
```

### `SingleArm._parse_vla_frames`

源码：[第 227 行](startouchclass.py#L227)。

```python
_parse_vla_frames(frames) -> Tuple[List[List[float]], List[float]]
```

**作用：** 内部静态方法：解析笛卡尔位姿与夹爪的组合帧。

| 参数 | 含义与单位 |
| --- | --- |
| `frames` | 位姿+夹爪组合帧，支持 (N,7)、单个 7 维点或规定字典。 |

**返回：** (poses, gripper_positions)，分别为 N×6 和 N 维列表。

**注意：** frames 支持单帧 7 数、N×7，或字典 {poses,gripper} / {cmdpos,cmdeulerrad,cmdgripper}。只检查部分维度与长度；不负责 IK，也不在此裁剪开度。详见组合帧格式。

**用法：**

```python
SingleArm._parse_vla_frames([[0.3, 0, 0.2, 0, 0, 0, 0.5]])
```

### `SingleArm._start_gripper_position_sync`

源码：[第 267 行](startouchclass.py#L267)。

```python
_start_gripper_position_sync(self, gripper_positions: List[float], duration_sec: Optional[float]) -> Tuple[threading.Event, Optional[threading.Thread]]
```

**作用：** 内部方法：启动后台线程按时间下发夹爪开度。

| 参数 | 含义与单位 |
| --- | --- |
| `gripper_positions` | 每个路点对应的归一化夹爪开度，N 个，0 闭合、1 打开。 |
| `duration_sec` | 内部夹爪同步总时长 s；None 使用每点 5 ms。 |

**返回：** (threading.Event, Thread 或 None)。

**注意：** gripper_positions 为 N 维开度，裁剪到 [0,1]；duration_sec 为总时长 s，None 时每点间隔 0.005 s。空列表不启动线程。使用 stop_event.set() 请求停止。此为 Python 定时近似同步，工作线程异常不会在这里转交主线程。

**用法：**

```python
stop_event, worker = arm._start_gripper_position_sync([0.0, 1.0], 2.0)
```

### `SingleArm.go_home`

源码：[第 311 行](startouchclass.py#L311)。

```python
go_home(self) -> bool
```

**作用：** 向六个关节发送全零角度的 3 秒规划目标。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 包装层固定返回 True。

**注意：** 无参数；home 固定是 [0,0,0,0,0,0] rad，不是用户校准参考姿态，也不是机械零位搜索。调用前确认全零姿态可达且路径无碰撞。

**用法：**

```python
arm.go_home()
```

### `SingleArm.gravity_compensation`

源码：[第 322 行](startouchclass.py#L322)。

```python
gravity_compensation(self) -> bool
```

**作用：** 调用底层重力补偿功能，供拖动示教等模式使用。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 包装层固定返回 True。

**注意：** 无参数；调用频率、是否持续运行、退出模式及负载补偿行为本文件未定义，需要结合底层实现和实机验证。

**用法：**

```python
arm.gravity_compensation()
```

### `SingleArm.set_joint`

源码：[第 328 行](startouchclass.py#L328)。

```python
set_joint(self, positions: Union[List[float], np.ndarray], tf: float=2.0) -> bool
```

**作用：** 设置单个关节目标，使用底层带时间规划接口。

| 参数 | 含义与单位 |
| --- | --- |
| `positions` | 6 维关节目标角，rad。 |
| `tf` | 单次规划时间，s；实际默认 2.0。 |

**返回：** 包装层固定返回 True。

**注意：** positions 为 6 维 rad；tf 默认 2.0 s（原注释写 4 秒，实际签名是 2 秒）。True 不表示已到位，需检查实际位置误差；此包装层不说明阻塞语义。

**用法：**

```python
arm.set_joint(q_target, tf=3.0)
```

### `SingleArm.set_joint_raw`

源码：[第 346 行](startouchclass.py#L346)。

```python
set_joint_raw(self, positions: Union[List[float], np.ndarray], velocities: Union[List[float], np.ndarray]) -> bool
```

**作用：** 透传关节目标位置和速度，用于外部周期伺服/遥操作。

| 参数 | 含义与单位 |
| --- | --- |
| `positions` | 6 维关节目标角，rad。 |
| `velocities` | 6 维关节目标速度，rad/s。 |

**返回：** 包装层固定返回 True。

**注意：** positions 为 6 维 rad；velocities 为 6 维 rad/s。没有 Python 轨迹规划、滤波、限速或形状检查；这些应由控制循环承担。零速度输入不等于机械臂不会运动。

**用法：**

```python
arm.set_joint_raw(q_target, [0.0]*6)
```

### `SingleArm.set_joint_waypoints`

源码：[第 363 行](startouchclass.py#L363)。

```python
set_joint_waypoints(self, waypoints: Union[List[List[float]], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None) -> float
```

**作用：** 执行整段关节路点轨迹，推荐用于回放。

| 参数 | 含义与单位 |
| --- | --- |
| `waypoints` | 关节路点，(N,6)，rad；允许单个 6 维点。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |

**返回：** 底层返回的 float 时长，单位 s。

**注意：** waypoints 为 N×6 rad，单点也可。time_sec 是整条轨迹请求总时长，不是每点时长。原 SDK 文档说明底层调用阻塞至完成/中断，当前状态自动作为起点。

**用法：**

```python
duration = arm.set_joint_waypoints(q_path, time_sec=6.0)
```

### `SingleArm.move_joint_waypoints`

源码：[第 376 行](startouchclass.py#L376)。

```python
move_joint_waypoints(self, waypoints: Union[List[List[float]], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None) -> float
```

**作用：** set_joint_waypoints 的兼容别名。

| 参数 | 含义与单位 |
| --- | --- |
| `waypoints` | 关节路点，(N,6)，rad；允许单个 6 维点。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |

**返回：** 与 set_joint_waypoints 相同。

**注意：** 参数和规则完全相同；新代码优先统一用 set_joint_waypoints。

**用法：**

```python
duration = arm.move_joint_waypoints(q_path, speed_percent=0.1)
```

### `SingleArm.move_joint_waypoints_with_gripper`

源码：[第 388 行](startouchclass.py#L388)。

```python
move_joint_waypoints_with_gripper(self, waypoints: Union[List[List[float]], np.ndarray], gripper_positions: Union[List[float], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None) -> float
```

**作用：** 执行关节路点及对应夹爪开度。

| 参数 | 含义与单位 |
| --- | --- |
| `waypoints` | 关节路点，(N,6)，rad；允许单个 6 维点。 |
| `gripper_positions` | 每个路点对应的归一化夹爪开度，N 个，0 闭合、1 打开。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |

**返回：** 底层/关节轨迹返回的时长，单位 s。

**注意：** waypoints 为 N×6，gripper_positions 展平成 N 维，长度必须一致；开度裁剪到 [0,1]。有原生接口时调用原生同步路径；缺少原生接口时先执行全部关节路径，再只发送最后一个夹爪值，中途夹爪不会同步。

**用法：**

```python
duration = arm.move_joint_waypoints_with_gripper(q_path, g_path, time_sec=6.0)
```

### `SingleArm.update_joint_waypoint_chunk`

源码：[第 415 行](startouchclass.py#L415)。

```python
update_joint_waypoint_chunk(self, waypoints: Union[List[List[float]], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None, switch_delay_sec: float=0.05) -> float
```

**作用：** 向底层提交新的关节轨迹块，用于滚动刷新轨迹。

| 参数 | 含义与单位 |
| --- | --- |
| `waypoints` | 关节路点，(N,6)，rad；允许单个 6 维点。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |
| `switch_delay_sec` | 传给底层的轨迹块切换延迟，s。 |

**返回：** 底层返回的 float；具体时间含义由原生接口决定。

**注意：** waypoints 为 N×6 rad；switch_delay_sec 默认 0.05 s，传给底层作为切换延迟。原生方法不存在时抛 RuntimeError。不能把它直接当作暂停/继续或硬实时抢占接口。

**用法：**

```python
arm.update_joint_waypoint_chunk(q_chunk, time_sec=0.5, switch_delay_sec=0.05)
```

### `SingleArm.update_joint_waypoint_chunk_with_gripper`

源码：[第 432 行](startouchclass.py#L432)。

```python
update_joint_waypoint_chunk_with_gripper(self, waypoints: Union[List[List[float]], np.ndarray], gripper_positions: Union[List[float], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None, switch_delay_sec: float=0.05) -> float
```

**作用：** 提交包含关节和夹爪的新轨迹块。

| 参数 | 含义与单位 |
| --- | --- |
| `waypoints` | 关节路点，(N,6)，rad；允许单个 6 维点。 |
| `gripper_positions` | 每个路点对应的归一化夹爪开度，N 个，0 闭合、1 打开。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |
| `switch_delay_sec` | 传给底层的轨迹块切换延迟，s。 |

**返回：** 底层返回的 float；具体含义依赖原生库。

**注意：** 参数同关节 chunk，另加 N 维 gripper_positions；长度必须与关节点数相等，开度裁剪 [0,1]。无原生方法则抛 RuntimeError，没有 Python 降级。

**用法：**

```python
arm.update_joint_waypoint_chunk_with_gripper(q_chunk, g_chunk, time_sec=0.5)
```

### `SingleArm.plan_joint_waypoints_with_gripper`

源码：[第 455 行](startouchclass.py#L455)。

```python
plan_joint_waypoints_with_gripper(self, q_start: Union[List[float], np.ndarray], waypoints: Union[List[List[float]], np.ndarray], gripper_positions: Union[List[float], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None)
```

**作用：** 向底层请求关节和夹爪轨迹的预规划结果。

| 参数 | 含义与单位 |
| --- | --- |
| `q_start` | 轨迹规划起始关节角，6 维 rad。 |
| `waypoints` | 关节路点，(N,6)，rad；允许单个 6 维点。 |
| `gripper_positions` | 每个路点对应的归一化夹爪开度，N 个，0 闭合、1 打开。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |

**返回：** 原生方法返回对象，Python 层未声明具体结构。

**注意：** q_start 为 6 维 rad；waypoints 为 N×6 rad；gripper_positions 为 N 维开度，裁剪 [0,1]。原生方法不存在时报错。本包装层没有发送 move 命令，但控制器初始化仍可能使能电机；结果字段不可凭名称臆测。

**用法：**

```python
plan = arm.plan_joint_waypoints_with_gripper(q_start, q_path, g_path, time_sec=6.0)
```

### `SingleArm.move_p`

源码：[第 481 行](startouchclass.py#L481)。

```python
move_p(self, poses: Union[List[List[float]], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None, blend_radius_m: float=0.002, position_tolerance_m: float=0.003, orientation_tolerance_rad: float=0.05) -> float
```

**作用：** 执行多点笛卡尔路径，支持拐角过渡；底层用 IK 转成关节路径。

| 参数 | 含义与单位 |
| --- | --- |
| `poses` | 笛卡尔路点，(N,6)，[x,y,z,roll,pitch,yaw]，m/rad。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |
| `blend_radius_m` | 笛卡尔拐角过渡半径，m。 |
| `position_tolerance_m` | 传给底层的路径位置容差，m。 |
| `orientation_tolerance_rad` | 传给底层的姿态容差，rad。 |

**返回：** 底层返回 float 时长，单位 s。

**注意：** poses 为 N×6；blend_radius_m 默认 0.002 m；position_tolerance_m 默认 0.003 m；orientation_tolerance_rad 默认 0.05 rad。容差参数传给底层，不能当成实测跟随误差保证。SDK 文档说明使用阻塞 waypoint 后端。

**用法：**

```python
duration = arm.move_p(poses, time_sec=3.0)
```

### `SingleArm.move_p_with_gripper`

源码：[第 500 行](startouchclass.py#L500)。

```python
move_p_with_gripper(self, frames, time_sec: Optional[float]=None, speed_percent: Optional[float]=None, blend_radius_m: float=0.002, position_tolerance_m: float=0.003, orientation_tolerance_rad: float=0.05) -> float
```

**作用：** 执行笛卡尔组合帧，联动末端路径和夹爪。

| 参数 | 含义与单位 |
| --- | --- |
| `frames` | 位姿+夹爪组合帧，支持 (N,7)、单个 7 维点或规定字典。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |
| `blend_radius_m` | 笛卡尔拐角过渡半径，m。 |
| `position_tolerance_m` | 传给底层的路径位置容差，m。 |
| `orientation_tolerance_rad` | 传给底层的姿态容差，rad。 |

**返回：** 底层或 move_p 返回的时长，单位 s。

**注意：** frames 见组合帧格式，其他参数同 move_p。优先原生接口；没有时启动 Python 夹爪线程再执行 move_p，结束后发送末个开度。时间模式按 time_sec 均分采样时刻；速度模式按每点 5 ms 下发，未按实际路径时长同步。

**用法：**

```python
duration = arm.move_p_with_gripper(frames, time_sec=3.0)
```

### `SingleArm.get_last_waypoint_command_samples`

源码：[第 544 行](startouchclass.py#L544)。

```python
get_last_waypoint_command_samples(self)
```

**作用：** 获取底层最近一次 waypoint 的命令采样结果，便于诊断/分析规划命令。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 原生采样对象；无此原生方法时返回 []。

**注意：** 无参数；是命令采样，不是实测关节轨迹。字段/列含义本包装层未定义，需查原生库或打印返回类型后适配。

**用法：**

```python
samples = arm.get_last_waypoint_command_samples()
```

### `SingleArm.move_l`

源码：[第 549 行](startouchclass.py#L549)。

```python
move_l(self, poses: Union[List[List[float]], np.ndarray], time_sec: Optional[float]=None, speed_percent: Optional[float]=None, blend_radius_m: float=0.0, position_tolerance_m: float=0.003, orientation_tolerance_rad: float=0.05) -> float
```

**作用：** 执行笛卡尔直线路径。

| 参数 | 含义与单位 |
| --- | --- |
| `poses` | 笛卡尔路点，(N,6)，[x,y,z,roll,pitch,yaw]，m/rad。 |
| `time_sec` | 整段请求时长，s；与正的 speed_percent 互斥。 |
| `speed_percent` | 速度比例 (0,1]，0.1 表示 10%，不是数值 10。 |
| `blend_radius_m` | 笛卡尔拐角过渡半径，m。 |
| `position_tolerance_m` | 传给底层的路径位置容差，m。 |
| `orientation_tolerance_rad` | 传给底层的姿态容差，rad。 |

**返回：** 底层返回 float 时长，单位 s。

**注意：** 参数同 move_p，但 blend_radius_m 默认 0。与 set_joint 不同，它用于约束末端路径；可达性、奇异点处理和容差验收依赖底层。

**用法：**

```python
duration = arm.move_l(poses, time_sec=3.0, blend_radius_m=0.0)
```

### `SingleArm.run_motion_program`

源码：[第 568 行](startouchclass.py#L568)。

```python
run_motion_program(self, program: Union[MotionProgram, List[Any]]) -> float
```

**作用：** 把 MotionProgram 或原生步骤列表交给底层执行。

| 参数 | 含义与单位 |
| --- | --- |
| `program` | MotionProgram 对象或原生 MotionProgramItem 列表。 |

**返回：** 底层返回 float 时长，单位 s。

**注意：** program 为 MotionProgram 或 list[Any]（原生 MotionProgramItem 列表），不是任意字典列表。SDK 文档说明该运动程序调用阻塞，包含 movej/movep/movel/sleep。

**用法：**

```python
duration = arm.run_motion_program(program)
```

### `SingleArm.set_end_effector_pose_euler`

源码：[第 577 行](startouchclass.py#L577)。

```python
set_end_effector_pose_euler(self, pos: Optional[Union[List[float], np.ndarray]]=None, euler: Optional[Union[List[float], np.ndarray]]=None, tf: float=2.0) -> bool
```

**作用：** 设置单个末端位置和欧拉角目标，调用带 tf 的底层位姿接口。

| 参数 | 含义与单位 |
| --- | --- |
| `pos` | 末端位置 [x,y,z]，m。 |
| `euler` | 末端 [roll,pitch,yaw]，rad。 |
| `tf` | 单次规划时间，s；实际默认 2.0。 |

**返回：** 包装层固定返回 True。

**注意：** pos 为 3 维 m，euler 为 3 维 rad；tf 实际默认 2.0 s。pos/euler 虽声明默认 None，但本层原样传给底层，不能假定省略就是保持不变。不能据此保证末端走直线。

**用法：**

```python
arm.set_end_effector_pose_euler(pos, rpy, tf=3.0)
```

### `SingleArm.set_end_effector_pose_euler_raw`

源码：[第 598 行](startouchclass.py#L598)。

```python
set_end_effector_pose_euler_raw(self, pos: Optional[Union[List[float], np.ndarray]]=None, euler: Optional[Union[List[float], np.ndarray]]=None) -> bool
```

**作用：** 透传单个末端位置和欧拉角目标，用于伺服更新。

| 参数 | 含义与单位 |
| --- | --- |
| `pos` | 末端位置 [x,y,z]，m。 |
| `euler` | 末端 [roll,pitch,yaw]，rad。 |

**返回：** 包装层固定返回 True。

**注意：** pos 为 3 维 m，euler 为 3 维 rad；没有 tf 参数。默认 None 的语义依赖底层；应用层应明确传完整目标并限制目标变化。

**用法：**

```python
arm.set_end_effector_pose_euler_raw(pos, rpy)
```

### `SingleArm.set_end_effector_pose_quat`

源码：[第 618 行](startouchclass.py#L618)。

```python
set_end_effector_pose_quat(self, pos: Optional[Union[List[float], np.ndarray]]=None, quat: Optional[Union[List[float], np.ndarray]]=None, tf: float=2.0) -> bool
```

**作用：** 将 wxyz 四元数转换为欧拉角后，调用带时间的末端位姿接口。

| 参数 | 含义与单位 |
| --- | --- |
| `pos` | 末端位置 [x,y,z]，m。 |
| `quat` | 4 维四元数；通常 [w,x,y,z]，xyzw 转换函数例外。 |
| `tf` | 单次规划时间，s；实际默认 2.0。 |

**返回：** 包装层固定返回 True。

**注意：** pos 为 3 维 m；quat 为 [w,x,y,z] 单位四元数；tf 默认 2.0 s。quat 虽默认 None，实际会立即解包转换，因此必须提供有效四元数。

**用法：**

```python
arm.set_end_effector_pose_quat(pos, quat, tf=3.0)
```

### `SingleArm.set_end_effector_pose_quat_raw`

源码：[第 631 行](startouchclass.py#L631)。

```python
set_end_effector_pose_quat_raw(self, pos: Optional[Union[List[float], np.ndarray]]=None, quat: Optional[Union[List[float], np.ndarray]]=None) -> bool
```

**作用：** 将 wxyz 四元数转为欧拉角，透传末端目标。

| 参数 | 含义与单位 |
| --- | --- |
| `pos` | 末端位置 [x,y,z]，m。 |
| `quat` | 4 维四元数；通常 [w,x,y,z]，xyzw 转换函数例外。 |

**返回：** 包装层固定返回 True。

**注意：** 参数同上一接口但无 tf；quat 实际必填。该接口仍经过四元数到欧拉角转换，不是四元数直接下发到底层。

**用法：**

```python
arm.set_end_effector_pose_quat_raw(pos, quat)
```

### `SingleArm.get_joint_positions`

源码：[第 644 行](startouchclass.py#L644)。

```python
get_joint_positions(self) -> np.ndarray
```

**作用：** 读取六个关节的实际位置。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 底层数组，按接口注解为 np.ndarray，预期形状 (6,)，rad。

**注意：** 无参数，不支持原注释所说的 joint_names 筛选。调用后建议检查长度与有限值。

**用法：**

```python
q = arm.get_joint_positions()
```

### `SingleArm.get_joint_velocities`

源码：[第 655 行](startouchclass.py#L655)。

```python
get_joint_velocities(self) -> np.ndarray
```

**作用：** 读取六个关节的实际速度。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 底层数组，预期形状 (6,)，rad/s。

**注意：** 无参数；反馈更新频率、新鲜度由底层决定，本层没有时间戳。

**用法：**

```python
dq = arm.get_joint_velocities()
```

### `SingleArm.get_joint_torques`

源码：[第 662 行](startouchclass.py#L662)。

```python
get_joint_torques(self) -> np.ndarray
```

**作用：** 读取六个关节的力矩反馈。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 底层数组，预期形状 (6,)，Nm。

**注意：** 无参数；电机/驱动反馈力矩不等于末端六维力，也没有在此扣除重力、摩擦和惯性。

**用法：**

```python
tau = arm.get_joint_torques()
```

### `SingleArm.get_ee_pose_quat`

源码：[第 668 行](startouchclass.py#L668)。

```python
get_ee_pose_quat(self) -> Tuple[np.ndarray, np.ndarray]
```

**作用：** 读取末端位置和欧拉角，再将姿态转成 wxyz 四元数。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** (pos, quat)，形状分别 (3,) 与 (4,)。

**注意：** 位置单位 m，quat 顺序 [w,x,y,z]；末端/TCP 与基坐标定义取决于底层机器人配置。

**用法：**

```python
pos, quat = arm.get_ee_pose_quat()
```

### `SingleArm.get_ee_pose_euler`

源码：[第 683 行](startouchclass.py#L683)。

```python
get_ee_pose_euler(self) -> Tuple[np.ndarray, np.ndarray]
```

**作用：** 读取末端位置和 roll/pitch/yaw。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** (pos, rpy)，每项预期形状 (3,)。

**注意：** 无参数；pos 单位 m，rpy 单位 rad；不要把末端读数当成相机或世界坐标系下的位姿。

**用法：**

```python
pos, rpy = arm.get_ee_pose_euler()
```

### `SingleArm.solve_ik`

源码：[第 693 行](startouchclass.py#L693)。

```python
solve_ik(self, pos, quat, q_seed=None)
```

**作用：** 求末端目标对应的关节角，不通过此方法直接发送运动目标。

| 参数 | 含义与单位 |
| --- | --- |
| `pos` | 末端位置 [x,y,z]，m。 |
| `quat` | 4 维四元数；通常 [w,x,y,z]，xyzw 转换函数例外。 |
| `q_seed` | 可选 IK 初始关节解，6 维 rad。 |

**返回：** (q, ok)：关节解和底层成功标志。

**注意：** pos 长度 3，quat 长度 4 且为 wxyz；q_seed 可选长度 6，rad，用于选择邻近解。长度错误抛 ValueError。先检查 ok，再检查关节变化和限位；底层配置可能允许近似 IK，ok=True 不保证位姿零误差。

**用法：**

```python
q, ok = arm.solve_ik(pos, quat, q_seed=arm.get_joint_positions())
```

### `SingleArm.openGripper`

源码：[第 711 行](startouchclass.py#L711)。

```python
openGripper(self) -> bool
```

**作用：** 请求夹爪完全打开。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 包装层固定返回 True。

**注意：** 无参数；最大开度取决于夹爪型号和配置，不保证瞬时到达。

**用法：**

```python
arm.openGripper()
```

### `SingleArm.closeGripper`

源码：[第 717 行](startouchclass.py#L717)。

```python
closeGripper(self) -> bool
```

**作用：** 请求夹爪闭合。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 包装层固定返回 True。

**注意：** 无参数；夹住物体不意味着位置一定达到闭合零点。

**用法：**

```python
arm.closeGripper()
```

### `SingleArm.setGripperPosition`

源码：[第 722 行](startouchclass.py#L722)。

```python
setGripperPosition(self, position: float) -> bool
```

**作用：** 设置归一化夹爪开度。

| 参数 | 含义与单位 |
| --- | --- |
| `position` | 归一化夹爪开度 [0,1]。 |

**返回：** 包装层固定返回 True。

**注意：** position 为开度，0=闭合、1=最大打开，无单位。本层直接传给底层，不自行校验或裁剪。

**用法：**

```python
arm.setGripperPosition(0.5)
```

### `SingleArm.setGripperDistance`

源码：[第 727 行](startouchclass.py#L727)。

```python
setGripperDistance(self, distance: float, kp: float=None, kd: float=None) -> bool
```

**作用：** 按物理开合距离控制夹爪，可选位置控制增益。

| 参数 | 含义与单位 |
| --- | --- |
| `distance` | 夹爪开合距离，m。 |
| `kp` | 可选底层夹爪位置控制 P 增益；不是夹持力。 |
| `kd` | 可选底层夹爪位置控制 D 增益。 |

**返回：** 包装层固定返回 True。

**注意：** distance 单位 m；kp、kd 默认 None，表示使用底层默认增益。本层仅在 kp 非 None 时传 kd；只传 kd 而不传 kp 会被忽略。范围及增益允许值依赖型号/底层配置。

**用法：**

```python
arm.setGripperDistance(0.03)
arm.setGripperDistance(0.03, kp=8.0, kd=0.1)
```

### `SingleArm.setGripperDistance_raw`

源码：[第 736 行](startouchclass.py#L736)。

```python
setGripperDistance_raw(self, distance: float, kp: float=8.0, kd: float=0.1) -> bool
```

**作用：** 用显式 kp/kd 调用 setGripperDistance 的兼容包装。

| 参数 | 含义与单位 |
| --- | --- |
| `distance` | 夹爪开合距离，m。 |
| `kp` | 可选底层夹爪位置控制 P 增益；不是夹持力。 |
| `kd` | 可选底层夹爪位置控制 D 增益。 |

**返回：** 包装层固定返回 True。

**注意：** distance 单位 m；kp 默认 8.0，kd 默认 0.1。此处 raw 没有独立原生透传调用，实际调用普通 setGripperDistance。

**用法：**

```python
arm.setGripperDistance_raw(0.03, kp=8.0, kd=0.1)
```

### `SingleArm.setGripperDistanceEffort`

源码：[第 740 行](startouchclass.py#L740)。

```python
setGripperDistanceEffort(self, distance: float, effort_nm: float) -> bool
```

**作用：** 设置物理夹爪开度及电机输出轴目标/限制力矩。

| 参数 | 含义与单位 |
| --- | --- |
| `distance` | 夹爪开合距离，m。 |
| `effort_nm` | 夹爪电机输出轴目标/限制力矩，Nm。 |

**返回：** 包装层固定返回 True。

**注意：** distance 单位 m；effort_nm 单位 Nm，两者均必填。需要底层及实际夹爪型号支持；不是指尖力 N 或独立力矩闭环。示例数值仅说明参数。

**用法：**

```python
arm.setGripperDistanceEffort(0.03, effort_nm=0.3)
```

### `SingleArm.setGripperPositionEffort`

源码：[第 745 行](startouchclass.py#L745)。

```python
setGripperPositionEffort(self, position: float, effort_nm: float) -> bool
```

**作用：** 设置归一化开度及电机目标/限制力矩。

| 参数 | 含义与单位 |
| --- | --- |
| `position` | 归一化夹爪开度 [0,1]。 |
| `effort_nm` | 夹爪电机输出轴目标/限制力矩，Nm。 |

**返回：** 包装层固定返回 True。

**注意：** position 为 [0,1] 开度；effort_nm 单位 Nm；底层/型号兼容性同上一接口。

**用法：**

```python
arm.setGripperPositionEffort(0.5, effort_nm=0.3)
```

### `SingleArm.setGripperAngle`

源码：[第 750 行](startouchclass.py#L750)。

```python
setGripperAngle(self, angle: float) -> bool
```

**作用：** 设置 TypeNex 从机械闭合点开始的总张开角。

| 参数 | 含义与单位 |
| --- | --- |
| `angle` | TypeNex 从闭合点起算的总张开角，rad。 |

**返回：** 包装层固定返回 True。

**注意：** angle 单位 rad，TypeNex 范围 [0,2.065597169735]；0 为闭合。不要套用内部电机角或其他型号标定偏置。

**用法：**

```python
arm.setGripperAngle(0.5)
```

### `SingleArm.setGripperAngleEffort`

源码：[第 755 行](startouchclass.py#L755)。

```python
setGripperAngleEffort(self, angle: float, effort_nm: float) -> bool
```

**作用：** 设置 TypeNex 总张开角及电机目标/限制力矩。

| 参数 | 含义与单位 |
| --- | --- |
| `angle` | TypeNex 从闭合点起算的总张开角，rad。 |
| `effort_nm` | 夹爪电机输出轴目标/限制力矩，Nm。 |

**返回：** 包装层固定返回 True。

**注意：** angle 为闭合点起算的 rad，effort_nm 为 Nm。此包装层没有 hasattr 兼容检测，旧版原生扩展缺少接口会报错。

**用法：**

```python
arm.setGripperAngleEffort(0.5, effort_nm=0.3)
```

### `SingleArm.setGripperAngleInterpolated`

源码：[第 760 行](startouchclass.py#L760)。

```python
setGripperAngleInterpolated(self, angle: float, duration: float=0.1, steps: int=20) -> bool
```

**作用：** 从当前 TypeNex 开度角出发，按绝对时间调度分步线性插值到目标角。

| 参数 | 含义与单位 |
| --- | --- |
| `angle` | TypeNex 从闭合点起算的总张开角，rad。 |
| `duration` | 夹爪插值总时长，s。 |
| `steps` | 夹爪插值步数，转为 int 后至少 1。 |

**返回：** 完成 Python 下发循环后返回 True。

**注意：** angle 必填，范围 [0,2.065597169735] rad 且有限；duration 默认 0.1 s，有限且非负；steps 默认 20，转 int 后必须≥1。此函数在 Python 中阻塞等待，duration=0 时仍循环发送 steps 次。循环结束不代表实测角已到位。

**用法：**

```python
arm.setGripperAngleInterpolated(0.5, duration=1.0, steps=50)
```

### `SingleArm.get_gripper_position`

源码：[第 786 行](startouchclass.py#L786)。

```python
get_gripper_position(self) -> float
```

**作用：** 读取归一化夹爪开度。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** float，0=闭合、1=最大打开。

**注意：** 无参数；不同型号的开度到物理距离换算由底层完成。

**用法：**

```python
g = arm.get_gripper_position()
```

### `SingleArm.get_gripper_distance`

源码：[第 790 行](startouchclass.py#L790)。

```python
get_gripper_distance(self) -> float
```

**作用：** 读取物理夹爪开合距离。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** float，单位 m。

**注意：** 原源码旁的 0/1 注释不适用于此方法；返回距离而非归一化比例。

**用法：**

```python
distance_m = arm.get_gripper_distance()
```

### `SingleArm.get_gripper_angle`

源码：[第 794 行](startouchclass.py#L794)。

```python
get_gripper_angle(self) -> float
```

**作用：** 读取 TypeNex 从机械闭合位置起算的总张开角。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** float，单位 rad。

**注意：** 无参数；不是电机原始转角，适用型号及版本要匹配。

**用法：**

```python
angle_rad = arm.get_gripper_angle()
```

### `SingleArm.get_gripper_effort`

源码：[第 798 行](startouchclass.py#L798)。

```python
get_gripper_effort(self) -> float
```

**作用：** 读取夹爪电机输出轴力矩反馈。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** float，单位 Nm。

**注意：** 无参数；不是指尖接触力。结合反馈有效性和时间判断能否使用该读数。

**用法：**

```python
effort_nm = arm.get_gripper_effort()
```

### `SingleArm.get_gripper_state`

源码：[第 802 行](startouchclass.py#L802)。

```python
get_gripper_state(self)
```

**作用：** 获取完整原生夹爪状态对象。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** 原生 GripperState 对象，不是字典。

**注意：** 无参数；字段表见下方。具体字段是否存在取决于已安装的原生扩展版本。

**用法：**

```python
state = arm.get_gripper_state()
print(state.feedback_valid, state.effort_nm)
```

### `SingleArm.cleanup`

源码：[第 806 行](startouchclass.py#L806)。

```python
cleanup(self)
```

**作用：** 调用底层清理接口，释放控制器资源。

**参数：** 无额外参数（`self` 为实例自身）。

**返回：** None。

**注意：** 无参数；Python 层会打印“销毁 SingleArm 对象”。在 finally 中调用；不能将其视为硬件急停或可恢复的暂停接口。

**用法：**

```python
arm.cleanup()
```

## 位姿与夹爪组合帧格式

`move_p_with_gripper()` 和 `_parse_vla_frames()` 支持以下格式，所有 N 必须一致。欧拉角均为 rad，位置为 m，夹爪为归一化开度。

```python
# 单帧或 N×7；最后一列是夹爪。
frames = [[0.30, 0.00, 0.20, 0.0, 0.0, 0.0, 0.5]]

frames = {
    "poses": [[0.30, 0.00, 0.20, 0.0, 0.0, 0.0]],
    "gripper": [0.5],
}

frames = {
    "cmdpos": [[0.30, 0.00, 0.20]],
    "cmdeulerrad": [[0.0, 0.0, 0.0]],
    "cmdgripper": [0.5],
}
```

此处坐标仅展示格式，不代表实机可达目标。公共调用优先使用规范的二维数组，避免不完整字典和高维数组触发索引/形状错误。

## GripperState 状态字段

下表来自同级 SDK 的 `README_API.md`；本仓库包装层原样返回原生对象，未在 Python 中定义字段。

| 字段 | 含义/单位 |
| --- | --- |
| `position` | 归一化夹爪开度 |
| `distance_m` | 开合距离，m |
| `motor_position_rad` | 电机位置，rad；与 TypeNex 总张开角不是同一概念 |
| `motor_velocity_rad_s` | 电机速度，rad/s |
| `effort_nm` | 电机输出轴力矩反馈，Nm |
| `commanded_effort_nm` | 指令力矩，Nm |
| `mos_temperature_c` | MOS 温度，℃ |
| `rotor_temperature_c` | 电机转子温度，℃ |
| `error_code` | 原生驱动状态/错误码，需按电机协议解读，不能假定非零均为故障 |
| `enabled` | 电机是否使能 |
| `feedback_age_ms` | 反馈年龄，ms |
| `feedback_valid` | 反馈是否有效 |
| `force_control_active` | 力位模式是否激活 |

## 与 01 / 03 集成的最小开发模式

以下片段仅说明调用顺序，`q_path`、`g_path`、`q_target` 应使用经过校验的实际数据；不是可直接运行的实机运动程序。

```python
# 01：周期伺服。q_target 已由应用层完成映射、滤波和限速。
arm.set_joint_raw(q_target, [0.0] * 6)
actual = arm.get_joint_positions()

# 03：整段关节/夹爪回放。time_sec 为整条路径时长。
if hasattr(arm.arm, "move_joint_waypoints_with_gripper"):
    duration = arm.move_joint_waypoints_with_gripper(q_path, g_path, time_sec=6.0)
else:
    # 本包装层的降级仅在关节运动结束后设置末个夹爪开度。
    # 若要途中同步，需要应用层实现并验证定时方案。
    raise RuntimeError("当前原生扩展不支持关节与夹爪同步回放")

# 任务编排：仅构建步骤；run_motion_program 才执行。
program = MotionProgram()
program.movej(q_path, time_sec=6.0).sleep(0.5)
duration = arm.run_motion_program(program)
```

## 开发时优先检查的实现差异

1. `set_joint` 和带 tf 的末端位姿方法实际默认 **2.0 秒**，不是旧注释中的 4 秒。
2. 单目标控制方法的 `True` 多为硬编码，不是到位反馈；底层返回值通常被丢弃。
3. `go_home()` 只是向六个零角度运动，不是校准、回零搜索或用户参考姿态。
4. `quaternion_to_euler_xyzw()` 不识别 wxyz；四元数控制接口必须提供有效 quat，即便签名默认是 None。
5. `setGripperDistance(kd=...)` 在 kp 未提供时忽略 kd；`setGripperDistance_raw()` 复用普通距离接口。
6. 两种夹爪降级不同：关节 waypoint 缺少原生接口时只在结尾设置夹爪；`move_p_with_gripper` 则使用 Python 线程近似同步。后者速度模式按 5 ms/点下发，不跟随真实轨迹时长。
7. 多数包装函数未检查形状、NaN/Inf、机械限位和反馈新鲜度，调用层需自行校验。
8. `plan_joint_waypoints_with_gripper()` 和命令采样接口的返回结构未在此文件定义，不能直接假定为某个字典字段集合。
9. 本文件没有公开的 pause/resume/stop 方法，也未定义完整的模式切换状态机；`cleanup()` 用于清理，硬件急停仍独立使用。
10. Python 文件版本号不代表原生扩展版本；`hasattr(arm.arm, "接口名")` 可检查部分能力，但不能替代签名/行为验证。

## 参考来源

- 主依据：[本仓库 startouchclass.py](startouchclass.py)。
- 底层语义补充：[同级 SDK API 文档](../startouch_sdk/README_API.md)。
- 夹爪力矩定义补充：[同级 SDK 夹爪说明](../startouch_sdk/GRIPPER_FORCE_CONTROL.md)。

本文只新增开发文档，没有修改 SDK 或现有控制脚本；未连接硬件。SDK 更新后应重新核对函数签名、原生能力和本文默认值。
