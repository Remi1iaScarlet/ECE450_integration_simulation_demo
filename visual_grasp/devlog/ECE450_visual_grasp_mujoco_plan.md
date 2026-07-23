# ECE 450 项目整理：OpenHarmony Robotic Arm + Visual Grasp 仿真迁移计划

> Version: 2026-06-26  
> Scope: 项目内容、个人职责、`visual_grasp` repo 的开发目标、近期 todo list 与汇报口径整理
>
> **进度更新（2026-06-29）**：仿真迁移最小闭环 **M1–M4 已完成**，**M5**（真·视觉抓取，`world_cam` Route B）**核心完成**——相机算出的 3D 点驱动机械臂抓起 cup 138mm，可被 OpenHarmony bridge 调用的 action API 待封装。在此基础上，**队友 runhanw（commit `636e2c7`）**推进 **Phase 7 眼在手 `wrist_cam`** + 实时 viewer，并写出多任务执行层规划 [`TODO_multitask_yolo_grasp.md`](TODO_multitask_yolo_grasp.md)。详细执行日志见 [`DEVLOG.md`](DEVLOG.md)；本文档保留为背景 / 职责 / 计划 / 汇报口径。
>
> **进度更新（2026-06-30）**：多任务执行层 **M1–M5 全部完成且 vision 验证**（pick / place_at / place_into / clear_table，cup+bottle+bowl 都用 YCB 贴图模型被 YOLO 检出，两物体清进碗）。第 4.1 节的 **rule-based parser + bridge 接口已落地（runhanw 写，已并入 main）**：`multitask/nl.py`（中英文规则关键词 `text→ParsedTask→executor`，`run_nl.py` CLI）+ `multitask/bridge.py`（bridge-ready JSON 接口 `visual_grasp.bridge.v1`，给上层/OpenHarmony/ROS bridge 调用，backend 分 `sim_mujoco`/`real_piper` 占位）+ `sim_gt` 感知后端（无 YOLO 机器跑全仿真）。这正是「LLM keywords + visual_grasp」路线的 keyword 步骤（先规则、后可换 LLM）。下一步：接 OpenHarmony command bridge（Stage 3）+ 真机后端（`real_piper`）。

---

## 1. 项目总体定位

本项目是 ECE 450 Design Project：**OpenHarmony-based Robotic Arm Simulation and Intelligent Control System**。

项目不应被表述为单纯“做一个机械臂”，而应被定位为：

> 一个基于 OpenHarmony 的智能家居 / 具身智能控制框架，其中 OpenHarmony 作为上层用户交互与设备协调平台，机械臂作为下游可视化执行设备，用于验证跨系统命令传递、机器人控制和智能交互能力。

项目总体链路可以概括为：

```text
User / OpenHarmony Client
        ↓
Natural Language / UI Command
        ↓
Intent Parsing / LLM JSON Validator
        ↓
Command Bridge / Control Middleware
        ↓
ROS / ROS2 Robot Backend
        ↓
Motion Planning / IK / Servo Control
        ↓
MuJoCo Simulation or Physical Arm
        ↓
Camera / Joint State / Motion Evidence
```

---

## 2. 当前项目阶段

根据 DR1 和 DR2 的内容，项目可以分为四个阶段。

| Stage | 名称 | 目标 | 当前状态 |
|---|---|---|---|
| Stage 1 | OpenHarmony Robot Sim Environment | 准备 OpenHarmony Robot Sim、Docker/rootfs、模拟器运行环境 | 基本完成 |
| Stage 2 | Robotic Arm Simulation Backend | 跑通 MuJoCo + ROS2 + MoveIt2 + ros2_control 的机械臂后端 | 已完成 / 已验证 |
| Stage 3 | OpenHarmony-side Command Bridge | 证明命令从 OpenHarmony 侧发出，并能触发机械臂后端动作 | 下一阶段关键任务 |
| Stage 4 | Intelligent Visual Grasp / NLP Extension | 加入 LLM intent parsing、YOLO/RGB-D/IK、视觉抓取与任务扩展 | 当前开始设计与迁移 |

---

## 3. DR1 已有基础

DR1 阶段已经建立了一个稳定的机械臂仿真后端 baseline。

已验证内容包括：

- Headless MuJoCo 能加载 Franka Panda 机械臂模型；
- ROS2 control 控制链路已激活，包括 joint-state、arm、hand controllers；
- MoveIt2 和 MoveIt Servo 可用于 motion planning 与 Cartesian servo；
- `/stage2_arm/move` 服务封装了基础动作；
- 支持基础 primitive commands：
  - `status`
  - `ready`
  - `open`
  - `close`
  - `up`
- world camera 和 wrist camera 可用于视觉证据；
- 可以录制 MP4 运动视频作为 demo evidence。

这部分的意义是：

> Host-side robotic-arm backend 已经能运行、能控制、能被验证。

但当前边界是：

> 它还不能单独证明 OpenHarmony OS / OpenHarmony app 已经真正参与控制闭环。

---

## 4. DR2 设计方向

DR2 的重点从“后端能否跑通”转向：

1. 上层自然语言命令如何转为标准机器人动作；
2. 视觉抓取算法如何接入机械臂控制系统；
3. OpenHarmony 客户端、command bridge、控制后端、物理仿真之间如何形成系统架构。

DR2 中主要选择了两条设计路线：

### 4.1 LLM Command Module

候选方案包括：

| 方案 | 特点 |
|---|---|
| Rule-based parser | 关键词匹配，稳定但扩展性弱 |
| LLM intent classifier | 让 LLM 从固定 intent list 中选择动作 |
| LLM + JSON + validator | LLM 输出结构化 JSON，再由 validator 检查合法性 |

最终选择：

```text
LLM + JSON + validator
```

原因：

- 命令空间仍然限制在 fixed intents 内；
- 可以拒绝不合法或不清楚的命令；
- 方便记录 raw input → parsed intent → validation result → execution result；
- 后续可以扩展到 task library。

### 4.2 Arm Control Algorithm

候选方案包括：

| 方案 | 特点 |
|---|---|
| LLM keywords + visual_grasp | LLM 提取物体和动作关键词，visual_grasp 完成感知和抓取 |
| End-to-end VLA | 视觉语言动作模型直接输出动作策略，灵活但风险高 |
| Open-vocab grounding + IK | 语言 grounding + RGB-D pose + IK，泛化强但集成成本高 |

最终选择：

```text
LLM keywords + visual_grasp pipeline
```

原因：

- 比 VLA 更可控；
- 不需要训练大型模型；
- 与已有 `visual_grasp` repo 代码基础兼容；
- 更容易在 Design Expo 中稳定演示；
- 可以先做 bottle/cup 视觉抓取 baseline，再扩展 task library。

---

## 5. 相关链接对应关系

| 链接 | 对应项目部分 | 作用 |
|---|---|---|
| `https://github.com/skywalkertzh/GC1-ZhuangHanyang-OpenHarmony-with-Robotic-Arm` | 小组主项目仓库 | ECE 450 项目的集成仓库，包含 OpenHarmony + robotic arm simulation/control 的主要实现 |
| `https://gitcode.com/openharmony-robot` | OpenHarmony Robot 上游生态 | 项目背景和官方具身智能操作系统生态依据 |
| `https://gitcode.com/openharmony-robot/docs/blob/main/device-dev/docker-build.md` | 官方编译环境 / 预编译镜像文档 | 对应 Stage 1 环境准备，说明 Docker、rootfs、image 等基础运行条件 |
| `https://gitcode.com/openharmony-robot/oh_robot_sim/blob/main/QUICKSTART.md` | 官方 OpenHarmony Robot Sim 运行指南 | 对应官方模拟器和具身仿真器 baseline |
| `https://github.com/runhanw/visual_grasp` | 视觉抓取子模块 baseline | 对应 DR2 中的 `LLM keywords + visual_grasp` 路线，为视觉检测、RGB-D 定位、IK 抓取提供原始代码框架 |

---

## 6. 我的职责定位

我的职责不应泛泛表述为“做仿真”或“做机械臂”，更准确的定位是：

> 负责机器人后端仿真集成、visual_grasp 到 MuJoCo 的迁移验证、坐标系对齐、仿真视觉抓取闭环，以及为 OpenHarmony command bridge 提供稳定可调用的机器人执行后端。

可以用英文汇报为：

```text
My responsibility focuses on robotic-arm backend integration and simulation-side visual grasp migration. Specifically, I work on importing the Piper arm model into MuJoCo, connecting simulated camera outputs to the YOLO-based visual_grasp pipeline, validating camera-to-base coordinate transforms, and testing whether a purely simulated environment can reproduce the bottle/cup grasping baseline before integrating it with the OpenHarmony-side command bridge.
```

### 6.1 我主要负责的模块

| 模块 | 我的职责 |
|---|---|
| MuJoCo simulation | 加载机械臂模型、搭建仿真场景、相机、物体 |
| Piper model migration | 将 Piper URDF/Xacro 或等价模型导入仿真器 |
| Visual grasp adaptation | 把真实 RealSense 输入替换为 MuJoCo camera 输出 |
| Coordinate transform | 处理 camera frame、link6 frame、base frame、world frame 之间的转换 |
| Backend verification | 生成可验证日志、截图、视频、运动结果 |
| Future OpenHarmony integration | 为 OpenHarmony command bridge 提供可调用的抓取 / 动作接口 |

### 6.2 我暂时不是主要负责的模块

| 模块 | 说明 |
|---|---|
| OpenHarmony UI / APP | 可能由其他成员负责，我只提供后端接口支持 |
| OpenHarmony 官方环境维护 | 不是主责，只需理解其在项目中的位置 |
| 纯 LLM 模型开发 | 不负责训练模型，只需配合 command format / task interface |
| VLA 模型训练 | 当前不建议作为主路线 |
| 真实 Piper 硬件调试 | 可能后续配合，但当前优先级是仿真环境迁移 |

---

## 7. `visual_grasp` repo 当前内容

`visual_grasp` repo 是一个基于 RealSense RGB-D 相机和 Piper 机械臂的视觉抓取 demo。

它当前实现的是一个 rule-based visual grasp baseline：

```text
RealSense RGB-D Camera
        ↓
Color Image + Depth Image
        ↓
YOLO Object Detection
        ↓
ROI Point Cloud Extraction
        ↓
3D Object Center Estimation
        ↓
ROS topic: /object_point
        ↓
Camera Frame → Piper Base Frame
        ↓
Piper Inverse Kinematics
        ↓
Piper SDK / CAN Control
        ↓
Move to object + close gripper
```

当前 repo 的主要能力：

- 使用 RealSense 采集 RGB + depth；
- 使用 `yolo11n.pt` 做目标检测；
- 当前主要检测 `bottle` 和 `cup`；
- 从 YOLO bbox 中截取 ROI depth；
- 根据相机内参将 depth pixel 反投影为 3D point cloud；
- 计算目标中心点；
- 发布 ROS topic `/object_point`；
- 使用 Piper DH 参数和手眼外参完成坐标变换；
- 使用 Piper IK 求解目标关节角；
- 通过 Piper SDK 控制真实机械臂执行抓取。

---

## 8. `visual_grasp` repo 关键文件说明

| 文件 | 作用 |
|---|---|
| `README.md` | 项目说明、流程、运行方式、常见问题 |
| `requirements` | Python 依赖列表，但只包含部分依赖 |
| `yolo11n.pt` | YOLO 目标检测权重 |
| `test_yolo.py` | 单独测试 YOLO 能否运行 |
| `test_realsense.py` | 单独测试 RealSense RGB-D 输出 |
| `test_depth_2_pointcloud.py` | 测试深度图转点云并发布到 RViz |
| `realsense_yolo_pc_roi.py` | 感知主程序：RealSense + YOLO + ROI point cloud + `/object_point` |
| `grasp_action.py` | 抓取执行程序：订阅目标点、坐标变换、IK、控制 Piper |
| `piper_arm.py` | Piper 运动学模型：DH 参数、FK、IK、关节限制、手眼外参 |
| `piper_tf_publisher.py` | 发布 Piper 机械臂 TF，包括 base、links、gripper、camera |
| `hand_eye_calibration.py` | 真机环境下的手眼标定程序 |
| `config/piper_description.urdf` | Piper 机械臂 URDF 描述文件 |
| `config/piper_description.xacro` | Piper 机械臂 Xacro 描述文件 |
| `config/*.rviz` | RViz 可视化配置 |

---

## 9. `visual_grasp` 当前局限

当前 repo 是一个基础 demo，而不是完整智能抓取系统。

主要局限：

1. **依赖 Piper SDK 和真机 CAN 环境**
   - 默认使用 `can0`；
   - 需要 Piper 机械臂供电、使能、急停释放；
   - 不适合直接在纯仿真环境中运行。

2. **依赖 RealSense RGB-D 相机**
   - 需要 USB 3.0；
   - 需要 `pyrealsense2`；
   - 真机相机和仿真相机接口不同。

3. **代码偏 ROS1 / rospy**
   - 当前使用 `rospy`；
   - 主项目偏 ROS2 / MuJoCo / MoveIt2；
   - 后续需要接口迁移。

4. **检测类别写死**
   - 当前主要检测 `bottle` 和 `cup`；
   - 还没有 task library；
   - 没有自然语言到多动作序列的完整映射。

5. **抓取策略较简单**
   - 取目标中心点附近作为抓取点；
   - 末端姿态固定；
   - 适合 baseline demo，但不是复杂 grasp planning。

6. **真机需要手眼标定**
   - 相机到末端 link6 的外参会影响抓取精度；
   - 仿真环境中可以直接从 MuJoCo frame / site / camera pose 获取外参，但仍需要坐标系方向严格对齐。

---

## 10. 当前开发目标

当前我的核心开发目标是：

> 把 `visual_grasp` 的 RealSense + YOLO + Piper IK 抓取 baseline 迁移到 MuJoCo 仿真环境中，先验证纯仿真环境能否完成 “看到瓶子 / 杯子 → YOLO 检测 → 得到目标 3D 点 → 坐标转换 → 机械臂移动 → 夹爪闭合” 的最小闭环。

### 10.1 第一阶段目标：仿真模型对齐

目标：让仿真器中的机械臂与未来真机 Piper 对齐。

需要完成：

- 找到并检查 Piper 的 URDF / Xacro；
- 确认 mesh、joint、link、limit、collision、inertial 信息是否完整；
- 将 Piper 模型导入 MuJoCo；
- 确认关节顺序、关节方向、关节限制与 `piper_arm.py` 一致；
- 确认 gripper / camera mount 位置合理。

成功标准：

```text
MuJoCo 可以加载 Piper 机械臂模型；
关节结构正确；
机械臂能在仿真中运动；
相机可以被固定在末端或合理视角处。
```

### 10.2 第二阶段目标：仿真视觉输入接入 YOLO

目标：用 MuJoCo camera 替换 RealSense camera。

需要完成：

- 在 MuJoCo 场景中添加 bottle / cup / table；
- 渲染 RGB 图像；
- 将 MuJoCo RGB frame 输入现有 YOLO pipeline；
- 测试 YOLO 是否能识别仿真中的 bottle / cup；
- 如果 YOLO 识别失败，调整物体模型、贴图、光照、相机角度，或者先使用更容易被 COCO YOLO 识别的物体模型。

成功标准：

```text
MuJoCo camera image 中的 bottle / cup 能被 YOLO 检测到，输出 bbox 和 confidence。
```

### 10.3 第三阶段目标：仿真 3D 目标点获取

目标：得到目标在机械臂基坐标系下的三维位置。

有两条路线：

#### 路线 A：快速验证路线

```text
直接读取 MuJoCo object body/site position
        ↓
作为 ground-truth object point
        ↓
转换到 base_link frame
```

优点：

- 简单、稳定；
- 适合先验证控制闭环；
- 不会被 depth buffer、相机内参、反投影误差阻塞。

#### 路线 B：完整视觉路线

```text
MuJoCo RGB image
        ↓
YOLO bbox
        ↓
MuJoCo depth image
        ↓
ROI depth point cloud
        ↓
object center under camera frame
        ↓
base frame transform
```

优点：

- 更接近 RealSense 真机 pipeline；
- 更适合最终展示 visual grasp 的完整性。

建议顺序：

```text
先做路线 A，验证机器人能移动；
再做路线 B，恢复完整视觉深度流程。
```

### 10.4 第四阶段目标：仿真抓取闭环

目标：在 MuJoCo 中完成最小抓取演示。

最小 demo：

```text
Piper arm in MuJoCo
        ↓
Simulated camera sees bottle
        ↓
YOLO detects bottle
        ↓
Get bottle 3D position
        ↓
Transform to base frame
        ↓
IK / controller computes action
        ↓
Arm moves near bottle
        ↓
Gripper closes
```

成功标准：

- 有终端日志；
- 有 YOLO 检测截图；
- 有坐标转换打印结果；
- 有机械臂运动视频；
- 有至少一次完整 bottle/cup grasp attempt。

---

## 11. 坐标系对齐问题

这个模块的核心技术难点之一是坐标系对齐。

真机中需要手眼标定：

```text
link6_T_camera
```

也就是相机相对于机械臂末端 link6 的位姿。

在 `visual_grasp` 中，抓取点转换逻辑本质是：

```text
object_point_camera
        ↓ link6_T_camera
object_point_link6
        ↓ base_T_link6
object_point_base
```

即：

```text
base_T_object = base_T_link6 × link6_T_camera × camera_T_object
```

在 MuJoCo 中，理论上可以直接读取：

```text
world_T_base
world_T_link6
world_T_camera
world_T_object
```

所以可以构造：

```text
base_T_camera = inverse(world_T_base) × world_T_camera
base_T_object = inverse(world_T_base) × world_T_object
```

或者，如果使用 camera frame 下的 depth 反投影结果：

```text
base_T_object = base_T_camera × camera_T_object
```

注意：

> MuJoCo camera frame、OpenGL camera frame、ROS optical frame 的轴方向可能不一致。必须明确 x/y/z 方向，否则 3D 点转换会错。

---

## 12. 当前 Todo List

### 12.1 P0：必须先完成

| 优先级 | 任务 | 目的 | 产物 |
|---|---|---|---|
| P0 | 找到 `piper_description.urdf` / `.xacro` | 确认 Piper 模型来源 | 文件路径记录 |
| P0 | 检查 mesh 资源是否完整 | 防止 MuJoCo 加载失败 | mesh list |
| P0 | 尝试 URDF/Xacro → MuJoCo MJCF | 将 Piper 放进仿真器 | `piper.xml` 或等价 MJCF |
| P0 | 在 MuJoCo 中加载 Piper | 验证模型可运行 | screenshot / log |
| P0 | 检查 joint order / limits | 确认与 Piper IK 对齐 | joint mapping table |
| P0 | 添加 bottle/cup/table 场景 | 建立视觉抓取环境 | scene XML |
| P0 | 添加 wrist camera 或 fixed camera | 提供图像输入 | camera frame screenshot |

### 12.2 P1：视觉接入

| 优先级 | 任务 | 目的 | 产物 |
|---|---|---|---|
| P1 | 渲染 MuJoCo RGB 图像 | 替代 RealSense color frame | RGB image |
| P1 | 将 RGB image 输入 YOLO | 验证 detection 能否工作 | annotated image |
| P1 | 调整物体贴图 / 光照 / 视角 | 提升 YOLO 识别成功率 | revised scene |
| P1 | 记录 bbox + confidence | 作为检测证据 | detection log |

### 12.3 P2：三维定位与坐标转换

| 优先级 | 任务 | 目的 | 产物 |
|---|---|---|---|
| P2 | 读取 MuJoCo object pose | 先用 ground truth 验证控制闭环 | object position log |
| P2 | 获取 camera pose / base pose | 建立 frame transform | transform matrix |
| P2 | 实现 `camera → base` 转换 | 替代真机手眼标定 | transform function |
| P2 | 打印并验证 target point | 检查坐标合理性 | debug log |
| P2 | 可选：读取 depth buffer | 恢复 RGB-D pipeline | depth image |
| P2 | 可选：实现 bbox ROI depth 反投影 | 接近 RealSense pipeline | point cloud / center |

### 12.4 P3：运动执行与抓取

| 优先级 | 任务 | 目的 | 产物 |
|---|---|---|---|
| P3 | 将 target point 送入 IK / controller | 计算机械臂动作 | joint target |
| P3 | 控制 MuJoCo Piper 移动到目标附近 | 验证运动执行 | motion video |
| P3 | 实现夹爪闭合 | 完成抓取动作 | gripper sequence |
| P3 | 记录一次完整 attempt | 作为 DR / Expo 证据 | log + video |

### 12.5 P4：后续扩展

| 优先级 | 任务 | 目的 |
|---|---|---|
| P4 | 扩充动作：move above、approach、grasp、lift、release | 形成基础 action library |
| P4 | 支持更多 object class | 从 bottle/cup 扩展到 pen/box 等 |
| P4 | 接入 LLM JSON command | 从自然语言进入 visual_grasp |
| P4 | 接入 OpenHarmony command bridge | 完成系统主线闭环 |
| P4 | 对比仿真与真机外参 | 支撑 sim-to-real 对齐 |

---

## 13. 建议开发顺序

不要一开始追求完整系统，应按最小可验证闭环推进。

推荐顺序：

```text
1. MuJoCo 成功加载 Piper
2. Piper joints 能动
3. 场景中出现 bottle / cup
4. MuJoCo camera 能截图
5. YOLO 能检测仿真 bottle / cup
6. 先直接读取 MuJoCo object pose
7. 完成 world/base/camera/object 坐标转换
8. 用目标点驱动机械臂靠近物体
9. 实现夹爪闭合
10. 再恢复 depth-based ROI point cloud
11. 再考虑 task library 和 OpenHarmony command bridge
```

---

## 14. 阶段性 Milestones

### Milestone 1：Piper in MuJoCo

交付物：

- Piper 模型成功加载截图；
- joint list / joint limit 表；
- 机械臂运动测试视频。

判断标准：

```text
仿真器中可以看到 Piper，且关节能按预期运动。
```

### Milestone 2：YOLO on MuJoCo Camera

交付物：

- MuJoCo camera RGB image；
- YOLO annotated image；
- bbox + confidence log。

判断标准：

```text
YOLO 能识别仿真画面中的 bottle / cup。
```

### Milestone 3：Coordinate Transform Verified

交付物：

- world_T_base / world_T_camera / world_T_object；
- base_T_object 计算结果；
- marker 或 debug visualization。

判断标准：

```text
目标点转换到 base frame 后位置合理，和 MuJoCo 场景一致。
```

### Milestone 4：Basic Simulated Grasp

交付物：

- 完整运行日志；
- 机械臂移动视频；
- 夹爪闭合视频；
- 成功 / 失败分析。

判断标准：

```text
仿真机械臂能基于目标点执行一次 bottle/cup grasp attempt。
```

### Milestone 5：OpenHarmony-ready Interface

交付物：

- action API table；
- command format；
- bridge input / output log；
- 可被 OpenHarmony command bridge 调用的 demo entry point。

判断标准：

```text
OpenHarmony 侧或上层 command bridge 可以触发仿真 visual grasp action。
```

### Milestone 状态与扩展（更新 2026-06-29）

| Milestone | 状态 | 证据 / 说明 |
|---|---|---|
| M1 Piper in MuJoCo | ✅ | joint 1:1 映射、FK 姿态一致 / 位置 ~10mm |
| M2 YOLO on MuJoCo camera | ✅ | cup conf 0.75（bottle domain-gap 延后） |
| M3 Coordinate transform | ✅ | `base_T_cup` 投影落在 YOLO bbox 内 |
| M4 Basic simulated grasp | ✅ | cup 抓起抬升 132mm |
| M5 OpenHarmony-ready interface | 🟡 核心完成 | 真·视觉抓取（`world_cam` Route B，相机驱动抓起 138mm）；可被 bridge 调用的 action API 待封装 |

**Phase 7（队友 runhanw，commit `636e2c7`，🚧 进行中）**：眼在手 `wrist_cam`（`link6_T_camera`，最贴近真机手眼标定）+ macOS 实时 viewer + 限类 YOLO（`classes=[bottle,cup]`）+ mug-like cup 模型 + 水平夹取；多任务执行层已写成 [`TODO_multitask_yolo_grasp.md`](TODO_multitask_yolo_grasp.md)（object registry + 原语 + task library `pick/place_at/place_on/place_into/clear_table` + FSM executor，先不做 NL/LLM/VLA）。

待办：① 本机 headless（`MUJOCO_GL=egl`）真渲染验证 wrist_cam 链路；② 修两个 bug——(a) wrist_cam 偶发「YOLO 没检出但仍抓到」，需确认喂给 YOLO 的确是 `wrist_cam` 画面、区分相机抓取与 `--fallback-ground-truth` 兜底；(b) 抓取姿态写死水平致部分 IK 无解，需让姿态可退化/自适应。详见 [`DEVLOG.md`](DEVLOG.md) 2026-06-29 条目第 8 节。

---

## 15. 汇报口径

### 15.1 中文汇报版

> 我目前负责的是把已有的 `visual_grasp` 视觉抓取 baseline 迁移到 MuJoCo 仿真环境中。原来的 repo 是 RealSense RGB-D 相机、YOLO 检测、ROI 点云、Piper IK 和 Piper SDK 真机控制组成的基础瓶子/杯子抓取框架。我的工作重点是先把 Piper 的 URDF/Xacro 模型导入 MuJoCo，使仿真环境和真机模型对齐；然后用 MuJoCo camera 替代 RealSense，把渲染图像接入 YOLO；接着处理 camera frame、base frame 和 object frame 之间的坐标转换；最后验证纯仿真环境中能否完成从 YOLO 识别到机械臂抓取的最小闭环。当前阶段还不急于扩充 task library，优先目标是证明仿真 visual_grasp pipeline 能跑通。

### 15.2 英文汇报版

```text
My current work focuses on migrating the existing visual_grasp baseline into the MuJoCo simulation environment. The original baseline uses a RealSense RGB-D camera, YOLO object detection, ROI point-cloud extraction, Piper inverse kinematics, and Piper SDK control to perform a simple bottle/cup grasping task. My task is to replace the real hardware input and output with simulation components: importing the Piper URDF/Xacro model into MuJoCo, attaching a simulated camera, feeding rendered images into YOLO, validating the camera-to-base coordinate transform, and testing whether a purely simulated Piper arm can reproduce the basic visual grasp pipeline. At this stage, the priority is not to build a full task library, but to prove the minimal simulated visual-grasp loop.
```

### 15.3 如果老师问“你现在做到哪一步？”

可以回答：

```text
I am currently working on the simulation-side migration of the visual_grasp baseline. The immediate milestone is to load the Piper model into MuJoCo and verify that simulated camera images can be passed into the YOLO detection pipeline. After that, I will validate the coordinate transform from camera frame to robot base frame and use the resulting target point to drive a basic simulated grasp attempt.
```

### 15.4 如果老师问“这个 repo 和 OpenHarmony 项目有什么关系？”

可以回答：

```text
The visual_grasp repository is not the full OpenHarmony system. It is the perception-to-action baseline for the selected DR2 arm-control concept. It provides the YOLO + RGB-D + IK logic for simple object grasping. Our project will adapt this baseline into the OpenHarmony robotic-arm system by replacing the real Piper/RealSense execution environment with a MuJoCo simulation first, and later exposing the grasp action through the OpenHarmony-side command bridge.
```

---

## 16. 风险与应对

| 风险 | 可能问题 | 应对策略 |
|---|---|---|
| URDF 转 MuJoCo 失败 | mesh、joint、mimic、collision 不兼容 | 先简化模型，只保留主要 link/joint；必要时手写 MJCF |
| YOLO 不识别仿真物体 | domain gap，仿真瓶子不像真实瓶子 | 使用真实贴图、COCO 风格物体、调整光照和相机角度 |
| 坐标转换错误 | MuJoCo camera frame 与 ROS optical frame 不一致 | 明确每个 frame 的轴方向，打印 transform matrix，使用 marker 验证 |
| IK 输出和仿真 joint 不匹配 | joint 顺序、方向、offset 不一致 | 建立 joint mapping table，对比 FK 结果 |
| 夹爪难以建模 | gripper mimic joint / contact 不稳定 | 第一版只做闭合动作，不要求稳定抓起；后续调 contact/friction |
| 和主项目 ROS2 不兼容 | `visual_grasp` 是 ROS1 / rospy | 第一版先独立仿真验证，后续再封装为 ROS2 service 或 bridge |
| Scope creep | 过早做 task library / LLM / 真机 | 坚持先完成 bottle/cup 最小闭环 |

---

## 17. 近期最小可交付成果

最小可交付成果不需要完整 task library，只需要证明以下链路：

```text
MuJoCo Piper arm scene
        ↓
Simulated camera RGB image
        ↓
YOLO detects bottle / cup
        ↓
Target 3D position obtained
        ↓
Coordinate transform to base frame
        ↓
Arm moves toward target
        ↓
Gripper closes
```

需要保存的 evidence：

- MuJoCo Piper 模型截图；
- 仿真相机原图；
- YOLO 检测标注图；
- 坐标转换日志；
- 机械臂移动视频；
- 抓取 attempt 成功 / 失败记录；
- 后续可被 OpenHarmony bridge 调用的 command entry point。

---

## 18. 当前最重要结论

当前项目主线应该保持清晰：

> DR1 已经证明 host-side robotic-arm backend 能跑；DR2 选择了 LLM keywords + visual_grasp 作为智能抓取扩展路线；我当前负责把 visual_grasp 从 RealSense + Piper 真机 baseline 迁移到 MuJoCo 仿真中，使仿真环境和未来真机 Piper 对齐，并验证 YOLO 识别、坐标转换和基础抓取闭环。

下一步最高优先级：

```text
Piper URDF/Xacro → MuJoCo
        ↓
MuJoCo camera → YOLO
        ↓
object pose / depth → base frame target
        ↓
simulated grasp attempt
```

不要过早扩展：

```text
task library
LLM multi-step planning
VLA
real hardware deployment
```

这些都应该在仿真 visual_grasp baseline 跑通之后再做。
