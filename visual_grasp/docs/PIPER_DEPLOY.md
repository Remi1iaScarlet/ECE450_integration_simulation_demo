# Piper 真机部署复制版

## 重要：连续轨迹后端当前验证状态（2026-07-19）

本仓库已新增 `multitask.piper_backend.PiperHardwareBackend`，用于把仿真侧已
验证的连续 joint-waypoint 执行语义同步到 Piper 控制边界。当前完成的是**控制
执行层、具体的依赖注入式 real executor 和 bridge 接口**，只经过 fake/stub 离线
单元测试；本轮没有连接
CAN、没有加载真实 SDK 执行动作，也没有做真机验证。不得把离线测试通过描述为
“真机可安全运行”或“真机验收完成”。

代码最低版本为 Python >= 3.10；本轮离线回归使用 Python 3.11.15。代码已使用
`X | None` 等 Python 3.10 语法；若目标机固定为 Python 3.8，本轮部署保持阻塞，
不能把本机 3.11 的结果外推为目标机或真机验证，也不在本轮做大规模 3.8 兼容降级。

当前边界如下：

- 复用 dependency-light `multitask.motion_plan.MotionPlan` 的
  `start_q -> segments -> waypoints[].q` 契约，执行时
  不重新求 IK、不跳过 waypoint；真实规划器应继续调用 `piper_arm.PiperArm.solve_ik`。
- 复用 `TrajectoryExecutionConfig`；按控制周期插值，使用新鲜关节反馈确认到位，
  固定 `sleep` 只用于节拍，不能作为到位判据。SDK 速度百分比另设独立保守上限，
  adapter 默认最多 20%；直接绕过 backend 调用 adapter 也会在 `ModeCtrl`/`JointCtrl`
  前拒绝六关节数量、非 finite、Piper 越限或超过 adapter hard cap 的 speed，不会 clamp。
- 未连接、未使能、陈旧/缺失/非法反馈、起点或 segment 接缝不一致、NaN、关节
  越限、请求或观测超速、following error、stop/fault、到位超时都会 fail closed。
- bridge 的 `armed=true` 与 API/CLI 的 `allow_hardware=true` 只负责签发一个不透明
  authorization capability；backend 的 `connect/enable/execute/gripper/stop` 和真实
  `PiperSdkAdapter` 的全部 SDK 路径都必须持有该 capability。没有 token 的 backend
  `connect()` 会在 adapter/CAN 构造前拒绝，不能用普通布尔参数绕过。
- enable 写入部分成功、反馈读取失败/超时或六电机使能未确认时，会 best-effort
  emergency stop 和 disconnect，并保留原始结构化错误；清理错误只作为附加详情。
- 相邻反馈 timestamp 必须严格递增。timestamp 未前进且位置变化会立即结构化拒绝；
  位置未变则继续等到 `feedback_timeout_s` 后拒绝，不能跳过 observed-speed guard。
- 轨迹在第一条 joint 命令前计算每段 nominal duration；按速度上限和控制周期必然
  超过 `waypoint_timeout_s` 的 plan 会原子拒绝，最终 sleep/read 后也复查 deadline。
- import、构造、bridge dry-run 不导入 `piper_sdk`，也不连接硬件。SDK 只会在
  显式 `connect()` 时延迟导入和构造。SDK 对象构造后的 required-method validation
  或 `ConnectPort()` 失败会 best-effort `DisconnectPort()`；backend connect 失败也会
  调用 adapter close，清理异常只附加到原始错误。
- 夹爪是独立显式命令，范围默认 `0.00~0.07 m`；`0.071 m` 及以上会在 adapter
  前拒绝。普通 `execute_motion_plan()` 永不自动闭合夹爪，只有显式
  `execute_grasp_plan()` 会在第一个 `LIFT` segment 前闭合一次。轨迹/夹爪异常会 best-effort
  stop，`close()` 仍会尝试断开，同时保留原始错误。

### 反馈 freshness 的默认 fail-closed 限制

当前 SDK 公共 joint message 只有 aggregate timestamp；任一 joint pair 都可能刷新它，
因此它不能证明三对关节都新鲜。adapter 仍同时验证 `joint_message.Hz`、
`status_message.time_stamp/Hz` 和 `GetArmEnableStatus()`，并通过
`AdapterCapabilities.per_pair_freshness` 明确暴露限制，但这些证据不能替代三对独立
timestamp。默认 `PiperSdkAdapter` 因而会返回 `per_pair_freshness=false`，真实 backend
在 enable 确认阶段以 `JOINT_FRESHNESS_UNPROVEN` fail closed 并回滚。

只有经特定 SDK/固件独立验收、能提供三对关节 timestamp 的集成，才可通过自定义
backend factory 向 `PiperSdkAdapter(pair_timestamp_reader=...)` 注入 reader。必须证明
三对 timestamp 每次都分别前进；aggregate timestamp 前进但任一 pair 未前进仍拒绝。
仓库默认 factory 不假设或伪造这项证据。

### 安全的第一阶段：只做 dry-run

下面命令只校验 bridge 命令，不构造 backend、executor 或 perception provider：

```bash
python3 -m multitask.bridge \
  '{"task":"pick","source":"cup","backend":"real_piper","perception":"yolo"}' \
  --dry-run
```

`real_piper` 明确拒绝 `sim_gt`。dry-run 成功只说明 JSON 和分发预览有效，不说明
感知、规划、SDK、CAN 或机械臂有效。

### 第二阶段：双重 armed 门和依赖注入

真实分发必须同时满足：

1. 命令 JSON 中 `"armed": true`；
2. API 参数 `allow_hardware=True`（CLI 对应 `--allow-hardware`）；
3. 配置真实 provider factory 和基于 `PiperArm`/`MotionPlan` builder seam 的 planner
   factory；backend 可注入，也可使用延迟加载 SDK 的默认 factory（但默认 adapter
   受上述 per-pair freshness 限制，不能进入真机运动）。

Python API 的集成形态是：

```python
run_bridge(
    command,
    allow_hardware=True,
    backend_factory=make_backend,
    perception_factory=make_realsense_yolo_provider,
    planner_factory=make_piper_motion_plan_builder,
)
```

当前 MuJoCo `TaskExecutor` 直接依赖仿真 world/model/data，不能原样用于真机。
仓库内的 `RealPiperTaskExecutor` 只串接
`provider.locate -> planner.build_pick_motion_plan -> backend.execute_grasp_plan`，不伪造
感知、IK、碰撞或 freshness。CLI/JSON 可用 `--provider-factory module:callable`、
`--planner-factory module:callable`（对应同名 JSON 字段）选择 factory；未配置真实
provider/planner 时，armed 调用会在 `connect()` 前明确失败并清理已构造资源。
bridge 会在 `connect()/enable()` 前调用 executor preflight，验证 provider/planner
方法、可选的依赖 preflight 和中性 MotionPlan contract；preflight 不做感知，也不生成
依赖当前关节状态的 plan。实际生成的 plan 会在执行前再次验证具体类型与结构。旧
`grasp_action.py` 仍是单点 `JointCtrl + 固定 sleep` 的旧入口，不属于本次连续
反馈后端，也不能作为本后端的真机验证证据。

仿真 bridge 现在与 executor CLI 使用相同的 `model.active -> scene -> analytic_ik ->
per-model overrides` 解析；相关回归通过注入 world/executor fake 运行，不启动 viewer。

### 真机验收前仍必须完成

- 锁定 Piper 固件与 `piper_sdk` 版本，逐项确认 `ModeCtrl`/`EmergencyStop`、
  状态码、joint/status Hz、三对关节独立反馈时间戳、使能反馈和单位换算；官方接口
  版本变化不能只靠离线 SDK-shaped stub 验证。没有三对 freshness 证据时保持默认拒绝。
- 在无负载、低速、具备物理急停和安全员的受控环境，依次验收 connect-only、
  enable/hold、单关节小步、连续 waypoint、反馈冻结、following error、fault 和
  emergency-stop/close 行为。
- 标定真实控制周期、允许 following error、到位容差、反馈最大年龄和机械臂/夹爪
  硬限位；默认值只是保守软件门，不是设备认证值。
- 接入并独立验证 RealSense/YOLO provider、真实场景碰撞/桌面净空来源以及真机
  executor 的 DETECT/PLAN/EXECUTE/VERIFY/RECOVER 行为。

离线回归命令（`tests/` 当前不是 Python package，因此必须使用 unittest discovery，
不要写成 `pytest` 或 package module 路径）：

```bash
python3 -m unittest discover -s tests -p 'test_piper_backend.py' -v
python3 -m unittest discover -s tests -p 'test_bridge.py' -v
```

本文件按实际运行顺序写。所有终端都在同一个项目目录下运行：

```bash
cd /home/zhou/wrh/visual_grasp
```

如果你之前 source 过旧版本脚本，先在当前终端执行一次，避免命令失败时终端退出：

```bash
set +e
```

## 0. 每个终端都先执行

每开一个新终端，先复制这一段：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
echo "$CONDA_DEFAULT_ENV"
echo "$CONDA_PREFIX"
which python3
python3 --version
```

预期类似：

```text
.conda_env
/home/zhou/wrh/visual_grasp/.conda_env
/home/zhou/wrh/visual_grasp/.conda_env/bin/python3
Python 3.10.x 或 Python 3.11.x
```

## 1. 一次性环境自检

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
echo "$CUDA_VISIBLE_DEVICES"
scripts/check_piper_env.sh
```

当前环境默认设置 `CUDA_VISIBLE_DEVICES=-1`，也就是 YOLO 走 CPU 推理。原因是本机 RTX 5070 的 `sm_120` 不被当前 PyTorch CUDA wheel 支持，强行走 GPU 会报 `no kernel image is available for execution on the device`。

全部通过时会看到：

```text
OK rospy
OK tf
OK sensor_msgs
OK geometry_msgs
OK visualization_msgs
OK pyrealsense2
OK ultralytics
OK piper_sdk
OK can
OK cv2
OK numpy
OK scipy
OK piper_description
```

YOLO 快速验证：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
bash scripts/yolo_smoke_test.sh
```

预期包含：

```text
detections 5
```

## 2. 配置 CAN

接好 Piper 供电，释放急停，插入 USB-CAN 后执行：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
sudo -E bash scripts/setup_can0.sh can0 1000000
ip -details link show can0
```

如果电脑只插了一个 USB-CAN，上面通常够用。

如果电脑插了多个 CAN 模块，先查 USB 地址：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
ip -br link show type can
sudo ethtool -i can0 | grep bus
```

假设查到的 `bus-info` 是 `1-2:1.0`，则这样绑定到 `can0`：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
sudo -E bash scripts/setup_can0.sh can0 1000000 "1-2:1.0"
ip -details link show can0
```

如果配置失败，看日志：

```bash
cd /home/zhou/wrh/visual_grasp
ls -lt .ros/log/setup_can0_*.log | head
tail -n 80 .ros/log/setup_can0_*.log
```

## 3. 单独测试 RealSense

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
python3 test_realsense.py
```

能看到 RealSense 彩色图和深度图窗口即可。按 `q` 或 `Esc` 退出。

## 4. 单独测试感知和点云

需要先启动 ROS master。

终端 1：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
roscore
```

终端 2：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
python3 realsense_yolo_pc_roi.py
```

终端 3，打开 RViz 看点云：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
rosrun rviz rviz -d config/depth_point_visual.rviz
```

检测到 `cup` 或 `bottle` 后，程序会发布：

```text
/object_point
/camera/depth/color/points
/camera/depth/color/points_roi
/object_center_marker
```

## 5. 完整抓取运行

完整运行建议开 5 个终端。每个终端都不要关。

### 终端 1：ROS master

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
roscore
```

### 终端 2：RealSense + YOLO 感知

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
python3 realsense_yolo_pc_roi.py
```

### 终端 3：Piper TF 发布

运行前确认 `can0` 已配置好：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
ip -details link show can0
python3 piper_tf_publisher.py
```

### 终端 4：RViz 可视化

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
roslaunch launch/piper_control.launch
```

### 终端 5：执行抓取

运行前确认机械臂周围安全、急停释放、无人手和障碍物在运动范围内。

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
ip -details link show can0
python3 grasp_action.py
```

`grasp_action.py` 会连接 `can0`，使能机械臂，订阅 `/object_point`，检测到 cup/bottle 后计算 IK 并执行一次抓取动作。

## 6. 常见问题命令

### 提示 ROS master 没启动

如果看到：

```text
Unable to register with master node [http://localhost:11311]
```

先开一个终端运行：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
roscore
```

### 检查当前 Python 环境

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
echo "$CONDA_DEFAULT_ENV"
echo "$CONDA_PREFIX"
which python3
python3 --version
```

### 检查 ROS topic

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
rostopic list
```

看目标点是否发布：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
rostopic echo /object_point
```

### 检查 CAN 是否存在

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
ip -br link show type can
ip -details link show can0
```

### 终端一失败就关闭

当前脚本已经修复，不会再自动开启 `set -e`。如果某个旧终端仍然会关闭，先执行：

```bash
set +e
```

或者重新开一个终端再执行：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
```

## 7. 安全检查

跑 `python3 grasp_action.py` 前必须确认：

- Piper 供电正常。
- 急停释放。
- USB-CAN 已配置为 `can0`，bitrate 为 `1000000`。
- RealSense 接 USB 3.0，彩色图和深度图稳定。
- 机械臂工作空间内没有人员、线缆和障碍物。
- `piper_arm.py` 中的 `link6_q_camera` 和 `link6_t_camera` 与当前相机安装位置一致；抓取偏差大时先重新做手眼标定。

## 8. 手眼标定

抓取位置不准时，优先做手眼标定。当前代码使用的是“固定棋盘格已知位置”的标定方式：

```text
base_link/world -> chessboard  已知，手动测量
camera -> chessboard           由 RealSense 图像 + solvePnP 求出
base_link -> link6             由 Piper 当前关节角 FK 求出
最终反算 link6 -> camera
```

### 8.1 准备棋盘格

当前脚本默认棋盘格参数在 `hand_eye_calibration.py`：

```python
self.pattern_size = (11, 8)   # 内角点数量，不是格子数量
self.square_size = 0.015      # 每个小方格边长，单位米
```

如果你的棋盘格不是 `11 x 8` 个内角点，或者方格不是 `15 mm`，先改这两个值。

### 8.2 测量棋盘格在 base_link 下的位置

修改 `hand_eye_calibration.py` 顶部的 `world_T_chessboard`：

```python
world_T_chessboard = np.array(
    [[0, -1, 0, 0.23],
     [-1, 0, 0, 0],
     [0, 0, -1, 0],
     [0, 0, 0, 1]])
```

这里的平移单位是米。最关键的是把棋盘格原点，也就是 OpenCV 检测到的第一个内角点，测到机械臂 `base_link` 坐标系下的位置。当前示例里的 `0.23` 只是旧实验位置，换了摆放就必须改。

### 8.3 手动移动机械臂并固定到标定姿态

推荐流程是先不要使能，让你手动把机械臂拖到一个相机能完整看到棋盘格的姿态；然后读取这个姿态；最后再使能并保持这个姿态。

先确认 `can0` 已配置：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
ip -details link show can0
```

然后手动移动机械臂到合适姿态。目标是：

- RealSense 能完整看到棋盘格
- 棋盘格不要太靠图像边缘
- 夹爪/机械臂不要遮挡棋盘格角点
- 标定过程中机械臂和棋盘格都不要再动

手动摆好后，读取当前角度，不会使能机械臂：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
python3 scripts/read_piper_pose.py
```

它会输出当前 6 个关节角，并给出类似命令：

```text
python3 scripts/move_piper_calib_pose.py --deg ... --speed 8
python3 scripts/enable_piper_current_pose.py --speed 8
```

如果你已经手动摆好了，直接运行下面命令使能并保持当前姿态：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
python3 scripts/enable_piper_current_pose.py --speed 8
```

这个终端保持打开。它会读取使能前的当前关节角，然后使能并持续发送这个姿态，方便你做标定。

如果你之后想让机械臂自动回到刚才读到的姿态，也可以复制 `read_piper_pose.py` 输出的 `move_piper_calib_pose.py --deg ...` 命令，例如：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
python3 scripts/move_piper_calib_pose.py --deg 0 10 -25 0 35 0 --speed 8
```

如果只想自动移动到位后退出：

```bash
python3 scripts/move_piper_calib_pose.py --deg 0 0 -20 0 30 0 --speed 10 --no-hold
```

### 8.4 运行标定

终端 1：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
roscore
```

终端 2：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
python3 hand_eye_calibration.py
```

终端 3，可选，用 RViz 看棋盘格点云和 TF：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
rosrun rviz rviz -d config/hand_eye_calibration.rviz
```

相机看到完整棋盘格后，终端 2 会反复输出：

```text
检测到棋盘格！
link6_T_cam
[[... ... ... ...]
 [... ... ... ...]
 [... ... ... ...]
 [0.  0.  0.  1. ]]
```

固定棋盘格和机械臂不动，等输出稳定后复制一组 `link6_T_cam`。

### 8.5 把矩阵转成 piper_arm.py 需要的 q/t

把下面命令里的矩阵替换成你终端输出的 `link6_T_cam`：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
python3 - <<'PY'
import numpy as np
from utils.utils_math import rotation_matrix_to_quaternion

T = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
], dtype=float)

q = rotation_matrix_to_quaternion(T[:3, :3])
t = T[:3, 3].tolist()
print("self.link6_q_camera =", q.tolist())
print("self.link6_t_camera =", t)
PY
```

然后把输出更新到 `piper_arm.py`：

```python
self.link6_q_camera = [...]
self.link6_t_camera = [...]
```

当前位置在 `piper_arm.py` 的 `PiperArm.__init__()` 里。

### 8.6 标定后验证

重新启动完整可视化：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
roscore
```

另开终端：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
python3 realsense_yolo_pc_roi.py
```

另开终端：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
python3 piper_tf_publisher.py
```

另开终端：

```bash
cd /home/zhou/wrh/visual_grasp
source scripts/piper_env.sh
roslaunch launch/piper_control.launch
```

在 RViz 里看目标点是否落在真实杯子/瓶子附近。确认目标点位置合理后，再运行 `python3 grasp_action.py`。
