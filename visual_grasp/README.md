# Visual Grasp

本项目是一个基于 RGB-D 相机和 Piper 机械臂的视觉抓取 demo。系统通过 RealSense 相机获取彩色图像和深度图像，使用 YOLO 检测杯子、瓶子等目标，再结合深度图计算目标在相机坐标系下的三维位置。随后系统将目标点从相机坐标系转换到机械臂基坐标系，通过 Piper 机械臂运动学求解关节角，并控制机械臂完成一个简单抓取动作。

这个项目当前定位是一个 rule-based 抓取 baseline，适合用于演示完整机器人视觉抓取流程，也适合作为后续 OpenHarmony 迁移工作的原始系统基础。

## MuJoCo bottle 网格抓取演示

macOS 上使用 `mjpython` 启动 MuJoCo Viewer。默认依次演示 5 个代表位置，
并运行当前 `piper_real + YOLO/RGB-D + bottle grasp profile` 抓取链。
默认演示场景保留蓝色 bottle、黄色罐子抓取目标和开放式矩形框，并使用连续限速轨迹：

```bash
cd /Users/wangrh/undergrad/大四/第三学期/毕设/visual_grasp
MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/mjpython \
  -m multitask.grasp_viewer
```

完整 5×4 网格：

```bash
MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/mjpython \
  -m multitask.grasp_viewer --preset full-grid --speed 2
```

自定义位置（`--position` 可重复）：

```bash
MUJOCO_GL=glfw /Users/wangrh/miniforge3/envs/visual-grasp-sim/bin/mjpython \
  -m multitask.grasp_viewer --position 0.34 -0.16 --position 0.38 -0.07
```

可用参数：`--yaw-deg`、`--backend yolo|sim_gt`、`--speed`、`--pause`、
`--hold`、`--view free|world|wrist`、`--scene-layout demo|isolated`以及
`--motion-mode continuous|legacy`。`sim_gt` 只是显式诊断模式，YOLO 失败时
程序不会自动回退。关闭 Viewer 窗口会终止剩余 cases。

## 1. 系统整体流程

完整流程如下：

```text
RealSense RGB-D 相机
    -> 彩色图像 + 深度图像
    -> YOLO 目标检测
    -> 根据检测框提取 ROI 深度点云
    -> 估计目标三维中心点
    -> 发布 ROS 目标点消息
    -> 机械臂读取当前关节状态
    -> 相机坐标系到机械臂基坐标系变换
    -> Piper 逆运动学求解
    -> 机械臂移动并闭合夹爪
```

主要 ROS 话题：

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | 完整深度点云 |
| `/camera/depth/color/points_roi` | `sensor_msgs/PointCloud2` | 目标检测框内的 ROI 点云 |
| `/object_point` | `geometry_msgs/PointStamped` | 相机坐标系下的目标中心点 |
| `/object_center_marker` | `visualization_msgs/Marker` | RViz 中显示目标中心点 |
| `/target_point_under_based` | `visualization_msgs/Marker` | 机械臂基坐标系下的目标点 |

## 2. 目录和文件说明

```text
visual_grasp/
├── README.md
├── requirements
├── yolo11n.pt
├── bus.jpg
├── Lecture3-visual grasp.pdf
├── test_yolo.py
├── test_realsense.py
├── test_depth_2_pointcloud.py
├── realsense_yolo_pc_roi.py
├── grasp_action.py
├── piper_arm.py
├── piper_tf_publisher.py
├── hand_eye_calibration.py
├── launch/
├── config/
└── utils/
```

### 2.1 主程序文件

| 文件 | 作用 |
| --- | --- |
| `realsense_yolo_pc_roi.py` | 感知主程序。启动 RealSense 相机，运行 YOLO 检测，提取检测框内深度点云，计算目标中心点，并发布 ROS 点云和目标点。 |
| `grasp_action.py` | 抓取执行程序。订阅 `/object_point`，读取 Piper 当前关节角，完成坐标变换、逆运动学求解和机械臂抓取控制。 |
| `piper_arm.py` | Piper 机械臂运动学模型。包含 DH 参数、正运动学、逆运动学、关节限制、相机到末端的手眼外参。 |
| `piper_tf_publisher.py` | 机械臂 TF 发布程序。读取 Piper 当前关节角，并在 ROS 中发布 `base_link`、`link1` 到 `link6`、夹爪和相机坐标系。 |
| `hand_eye_calibration.py` | 手眼标定程序。通过棋盘格和 RealSense 图像估计相机相对于机械臂末端的位姿。 |

### 2.2 测试脚本

| 文件 | 作用 |
| --- | --- |
| `test_yolo.py` | YOLO 单独测试脚本。读取 `bus.jpg`，使用 `yolo11n.pt` 进行目标检测，并显示检测结果。 |
| `test_realsense.py` | RealSense 单独测试脚本。打开相机，显示彩色图像和深度伪彩图，验证相机连接是否正常。 |
| `test_depth_2_pointcloud.py` | 深度图转点云测试脚本。将 RealSense 深度图反投影成点云，并通过 ROS 发布给 RViz 显示。 |

### 2.3 工具函数

| 文件 | 作用 |
| --- | --- |
| `utils/utils_math.py` | 数学工具函数，包括旋转矩阵、四元数、欧拉角之间的转换。 |
| `utils/utils_ros.py` | ROS 工具函数，包括发布 TF、发布目标点、发布球形 marker、发布轨迹等。 |
| `utils/utils_piper.py` | Piper 机械臂工具函数，包括使能机械臂、读取关节角等。 |

### 2.4 配置、模型和资源文件

| 文件或目录 | 作用 |
| --- | --- |
| `requirements` | Python 依赖列表。注意该文件只包含部分依赖，YOLO 和 RealSense 相关依赖需要额外安装。 |
| `yolo11n.pt` | YOLO 目标检测模型权重。 |
| `bus.jpg` | YOLO 测试图片。 |
| `Lecture3-visual grasp.pdf` | 课程或项目参考材料。 |
| `launch/piper_control.launch` | 启动 RViz 可视化配置的 ROS launch 文件。 |
| `config/depth_point_visual.rviz` | 用于查看相机坐标系下点云的 RViz 配置。 |
| `config/hand_eye_calibration.rviz` | 用于手眼标定过程可视化的 RViz 配置。 |
| `config/piper_ctrl.rviz` | 用于查看机械臂、TF 和目标点的 RViz 配置。 |
| `config/piper_description.urdf` | Piper 机械臂 URDF 描述文件。 |
| `config/piper_description.xacro` | Piper 机械臂 Xacro 描述文件。 |
| `__pycache__/` 和 `utils/__pycache__/` | Python 自动生成的缓存文件，不需要手动修改。 |

## 3. 运行环境准备

### 3.1 硬件要求

- RealSense RGB-D 相机。
- Piper 机械臂。
- 机械臂 CAN 通信环境，默认代码中使用 `can0`。
- 安装好 ROS 的 Linux 环境。

注意：

- RealSense 建议插到主板 USB 3.0 接口，否则可能无法稳定输出深度和彩色图像。
- Piper 机械臂运行前需要确认急停、供电、CAN 连接和机械臂活动空间安全。

### 3.2 软件依赖

项目依赖 ROS、OpenCV、RealSense SDK、YOLO、Piper SDK 等环境。

进入项目目录：

```bash
cd /home/ubuntu/arm_student_ws/project2
```

如果你的项目路径不同，请把上面的路径换成实际的 `visual_grasp` 所在目录。

安装基础依赖：

```bash
pip install -r requirements
```

安装 YOLO 依赖：

```bash
pip install ultralytics
```

安装 RealSense Python 依赖：

```bash
pip install pyrealsense2
```

如果缺少 OpenCV 或 matplotlib：

```bash
pip install opencv-python matplotlib
```

每个需要使用 ROS 的终端都需要先执行：

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
```

如果你的 ROS 工作空间路径不同，请改成自己的 `setup.bash` 路径。

## 4. 按模块测试

建议先按模块测试，确认相机、模型、点云和机械臂分别可用，再运行完整抓取流程。

### 4.1 测试 YOLO

作用：验证 `ultralytics` 和 `yolo11n.pt` 是否可以正常运行。

```bash
cd /home/ubuntu/arm_student_ws/project2
python test_yolo.py
```

预期结果：

- 程序读取 `bus.jpg`。
- 弹出 YOLO 检测结果窗口。

如果模型文件路径报错，请确认当前工作目录下存在 `yolo11n.pt`。

### 4.2 测试 RealSense

作用：验证 RealSense 相机是否可以正常输出彩色图和深度图。

```bash
cd /home/ubuntu/arm_student_ws/project2
python test_realsense.py
```

预期结果：

- 弹出 `RealSense` 窗口。
- 左侧显示彩色图像，右侧显示深度伪彩图。
- 按 `q` 或 `Esc` 退出。

如果无法打开相机，优先检查：

- 相机是否插在 USB 3.0 接口。
- 是否有其他程序占用了 RealSense。
- `pyrealsense2` 是否安装成功。

### 4.3 测试深度图转点云

终端 1：运行点云发布程序。

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
cd /home/ubuntu/arm_student_ws/project2
python test_depth_2_pointcloud.py
```

终端 2：打开 RViz 查看点云。

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
rosrun rviz rviz -d /home/ubuntu/arm_student_ws/project2/config/depth_point_visual.rviz
```

预期结果：

- RViz 中可以看到相机坐标系下的三维点云。
- 点云话题为 `/camera/depth/color/points`。

## 5. 运行目标检测 + ROI 点云

该步骤将 RealSense、YOLO、深度点云和目标 ROI 提取组合起来。

终端 1：运行感知主程序。

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
cd /home/ubuntu/arm_student_ws/project2
python realsense_yolo_pc_roi.py
```

终端 2：打开 RViz。

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
rosrun rviz rviz -d /home/ubuntu/arm_student_ws/project2/config/depth_point_visual.rviz
```

预期结果：

- OpenCV 窗口中可以看到 RealSense 图像和 YOLO 检测结果。
- RViz 中可以看到点云和目标中心 marker。
- 程序检测到 `bottle` 或 `cup` 时，会发布 `/object_point`。

当前检测逻辑在 `realsense_yolo_pc_roi.py` 中写死为：

```text
bottle 或 cup，且 confidence > 0.3
```

如果需要检测其他类别，需要修改 `YOLODetection()` 函数中的类别过滤条件。

## 6. 运行完整抓取流程

完整抓取流程需要多个终端同时运行。

### 6.1 终端 1：运行感知程序

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
cd /home/ubuntu/arm_student_ws/project2
python realsense_yolo_pc_roi.py
```

该程序负责：

- 采集 RealSense 图像。
- 运行 YOLO。
- 计算目标中心点。
- 发布 `/object_point`。

### 6.2 终端 2：发布 Piper 机械臂 TF

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
cd /home/ubuntu/arm_student_ws/project2
python piper_tf_publisher.py
```

该程序负责：

- 读取 Piper 当前关节状态。
- 发布 `base_link` 到各个机械臂 link 的 TF。
- 发布相机坐标系相对于末端的 TF。

### 6.3 终端 3：打开 RViz

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
cd /home/ubuntu/arm_student_ws/project2
roslaunch launch/piper_control.launch
```

该步骤用于查看：

- 机械臂模型。
- TF 坐标系。
- 相机点云。
- 目标点 marker。

### 6.4 终端 4：执行抓取

运行前请确认机械臂周围安全，没有人员或障碍物处在运动范围内。

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
cd /home/ubuntu/arm_student_ws/project2
python grasp_action.py
```

该程序负责：

- 连接 Piper 机械臂。
- 使能机械臂和夹爪。
- 订阅 `/object_point`。
- 读取当前关节角。
- 将目标点从相机坐标系转换到机械臂基坐标系。
- 调用 `piper_arm.py` 中的逆运动学。
- 控制机械臂移动到目标点附近。
- 闭合夹爪并回到预设位置。

## 7. 手眼标定

如果抓取位置偏差较大，需要重新进行手眼标定。标定结果会影响相机坐标系下目标点转换到机械臂基坐标系后的准确性。

### 7.1 标定前准备

1. 固定放置棋盘格。
2. 测量棋盘格左上角角点到机械臂基座中心的位移。
3. 将测量结果填入 `hand_eye_calibration.py` 顶部的 `world_T_chessboard`。

例如：

```python
world_T_chessboard = np.array(
    [[0, -1, 0, 0.23],
     [-1, 0, 0, 0],
     [0, 0, -1, 0],
     [0, 0, 0, 1]])
```

### 7.2 运行标定程序

终端 1：运行标定。

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
cd /home/ubuntu/arm_student_ws/project2
python hand_eye_calibration.py
```

终端 2：发布机械臂 TF。

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
cd /home/ubuntu/arm_student_ws/project2
python piper_tf_publisher.py
```

终端 3：打开 RViz。

```bash
source /home/ubuntu/arm_student_ws/piper_ros/devel/setup.bash
rosrun rviz rviz -d /home/ubuntu/arm_student_ws/project2/config/hand_eye_calibration.rviz
```

### 7.3 更新标定结果

查看 `hand_eye_calibration.py` 终端输出中的：

```text
link6_T_cam
```

将该变换结果转换为旋转和平移后，更新到 `piper_arm.py` 中：

```python
self.link6_q_camera = ...
self.link6_t_camera = ...
```

其中：

- `self.link6_q_camera` 是相机相对于 `link6` 的旋转四元数，顺序为 `[w, x, y, z]`。
- `self.link6_t_camera` 是相机相对于 `link6` 的平移，单位为米。

## 8. 关键代码逻辑说明

### 8.1 目标检测逻辑

`realsense_yolo_pc_roi.py` 中的 `YOLODetection()` 会对彩色图像运行 YOLO，并筛选目标类别：

```text
bottle
cup
```

检测结果为二维框：

```text
x1, y1, x2, y2
```

### 8.2 ROI 点云提取逻辑

`extract_roi_cloud()` 会根据 YOLO 检测框截取深度图 ROI，并把像素点反投影为三维点：

```text
x = (u - cx) * z / fx
y = (v - cy) * z / fy
z = depth / 1000.0
```

随后过滤无效深度点，并计算目标点云中心。

### 8.3 抓取执行逻辑

`grasp_action.py` 中的 `move_and_grasp()` 会执行：

```text
相机坐标系目标点
    -> link6_T_cam
    -> base_T_link6
    -> 机械臂基坐标系目标点
    -> 构造目标末端位姿
    -> 逆运动学求关节角
    -> 控制机械臂运动
    -> 闭合夹爪
```

当前抓取策略比较简单：目标姿态固定，抓取点为检测目标中心附近。这个策略适合演示基础流程，但不是复杂抓取算法。

## 9. 常见问题

### 9.1 YOLO 模型找不到

确认当前运行目录下有：

```text
yolo11n.pt
```

建议在 `visual_grasp` 目录下运行脚本。

### 9.2 RealSense 无法打开

检查：

- 相机是否连接到 USB 3.0 接口。
- 是否有其他程序占用相机。
- `pyrealsense2` 是否安装成功。
- RealSense 权限或驱动是否正常。

### 9.3 RViz 中没有点云

检查：

- 是否运行了点云发布程序。
- RViz 中订阅的话题是否为 `/camera/depth/color/points` 或 `/camera/depth/color/points_roi`。
- Fixed Frame 是否设置为 `camera`、`base_link` 或配置文件中对应的坐标系。

### 9.4 机械臂不动

检查：

- Piper 是否供电。
- CAN 设备是否为 `can0`。
- `piper_sdk` 是否安装成功。
- 机械臂是否使能。
- 急停是否释放。
- 运行 `grasp_action.py` 前是否有有效 `/object_point`。

### 9.5 抓取位置偏差大

优先检查：

- 手眼标定是否准确。
- `piper_arm.py` 中的 `self.link6_q_camera` 和 `self.link6_t_camera` 是否更新。
- RealSense 深度是否稳定。
- 目标是否被遮挡或反光。

## 10. 建议运行顺序

第一次运行时，建议按以下顺序逐步验证：

```text
1. python test_yolo.py
2. python test_realsense.py
3. python test_depth_2_pointcloud.py + RViz
4. python realsense_yolo_pc_roi.py + RViz
5. python piper_tf_publisher.py + RViz
6. python grasp_action.py
```

确认每一步都正常后，再进行完整抓取演示。
