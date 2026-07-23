# Visual Grasp — MuJoCo 迁移开发日志 (DEVLOG)

> 本文件是 `visual_grasp` 从「真机 RealSense + Piper(CAN)」迁移到「纯 MuJoCo 仿真」的开发日志。
> 每个条目按日期倒序追加，记录：当天目标、做了什么、证据、决策、下一步。
> 配套背景文档见 [`ECE450_visual_grasp_mujoco_plan.md`](ECE450_visual_grasp_mujoco_plan.md)。

---

## 2026-07-08 — IK 统一落地：piper_real 模型重搭 + 解析 IK 驱动 multitask（✅ 全链路通）

**目标**：接上 2026-06-30 的交接——在 DH 精确匹配的 `piper_real` 模型上重搭抓取仿真，用 `piper_arm.solve_ik` 取代 MuJoCo 数值 IK，让**仿真和真机共用一套 IK**。Menagerie 那套保持不动做 fallback。全程 headless `MUJOCO_GL=egl`。

### 已完成 ✅

**1. piper_real 模型（`sim/models/piper_real/`）**
- URDF→MJCF：MuJoCo 编译 `piper_real.urdf`（自带 `<mujoco>` 块）存 `piper_real_gen.xml`。实测 **DH-FK 与该模型 link6 body 位姿差 <0.09mm（位置+姿态都对齐）**——DH link6 帧 == 模型 link6 帧，`T_off` 是干净常量。
- `piper_real.xml`（可用机器人文件）：在 gen 基础上补 position actuator(j1-6+gripper) + 夹爪 equality(joint7/8) + home keyframe + **link6 上 tcp site(指垫中点 z=0.10)** + wrist_cam。关键补丁：① 所有臂 body 加 `gravcomp="1"`（否则位置伺服顶不住重力，落 ~36mm）；② `<exclude base_link↔link1>`（不排会卡死 joint1 base yaw）；③ 把 base_link 包成世界原点的独立 body（URDF 固定基座会把它并进 world，感知注册表需要 `world_T_base`）。
- `scene_multitask_real.xml`：include 机器人 + 桌子 + mug cup + YCB 贴图 bottle/bowl（共享 `../piper/assets/`，不复制）+ world_cam。

**2. 解析 IK 接入（`sim/grasp_sim.py:ik_target_analytic`，`use_analytic` 开关）**
- `T_off = inv(DH_link6(q)) @ site(q)` 实时读当前态算（自标定，实测 == `translate(0,0,0.10)`）；`T_target = M_site @ inv(T_off) @ translate(0,0,0.06)` 喂 `solve_ik`，warm-start 当前 qpos。
- **两个关键坑**：① `horizontal_grasp_frame` 让 link6 +x 朝上，需 ~180° 腕翻转（超 joint6 限位）→ 两指夹爪绕接近轴滚 180° 是同一个抓取，**试 roll∈{0,π} 取可达解**；② roll 选择必须**种子无关**（按 |joint6| 选），否则 pre-grasp 和 grasp 各自「就近种子」会选反 roll，腕部 180° 翻转扫飞物体。
- 位置-only（pre-grasp）解不出时退 MuJoCo 数值 IK 兜底。
- **实测解析 IK site 精度：collision-free 目标中位 0.09mm（亚毫米），y=0 目标 0.03mm**。IK 本身精确，误差全来自与桌面/物体的物理碰撞。

**3. 抓取/感知调参（真机 workspace 比 Menagerie 小 + 长指夹爪）**
- 真机够不到 <0.32m 半径的水平位姿 → pre-grasp backoff 0.10→**0.06**；水平位姿在 z=0.15 有死区 → 短物体（bottle）抬到 **z=0.165** 抓（`min_grasp_z_clearance=0.045`），恢复干净直入水平接近。
- bottle 用**干净圆柱碰撞**（不可见）+ 芥末贴图 mesh 做纯视觉（照搬 cup 套路），长指夹爪不再被不规则 mesh 卡住。
- **每次 pick 前先回 ready 位姿**（解析 IK 种子相关，clear_table 里放完 cup 从残余位姿起手会选到差分支）。
- **wrist_cam scan 位姿重扫**（Menagerie 值不迁移）：`[0.15,0.5,0,0,0,0,0.035]` + `[0.3,0.5,0,0,-0.2,0,0.035]`，每个都稳定检出 cup~0.68 / bottle~0.8 / bowl~0.9。
- **深度反投影半径修正 1.0→0.5**（`detector.radius_correction`，per-model）：wrist_cam 的 ROI 中值本就在物体内部，满修正过冲 → cup 定位误差 17mm→2.7mm、bottle 8.7→3mm。
- **grasp z 用桌面相对固定值**（`use_table_grasp_z`）：YOLO 的 z 抖动几 mm 会把抓取推进差构型（z=0.17 撞飞、0.165 稳），改用 table+clearance 固定高度。

### 验证（headless `MUJOCO_GL=egl`）

**sim_gt 后端 = 确定性验证路径，全绿且可复现**（DEMOS 里 sim_gt 本就是稳定验证后端）：

| 任务 | sim_gt（×2+ 复现一致） | YOLO（相机驱动） |
|---|---|---|
| pick cup | 127mm ✅ | 128mm ✅（稳） |
| pick bottle | 173mm ✅ | 174mm ✅（稳，standalone ×3 一致） |
| place_at cup | 29mm ✅ | — |
| place_into cup→bowl | 30mm ✅ | 28mm ✅（稳） |
| clear_table [cup,bottle]→bowl | 30/52mm ✅（×2 一致） | cup ✅ 稳；**bottle 边缘不稳**（见下） |

- 证据（`multitask/evidence/{pick_cup,place_into_cup,clear_table_cup}_real{,_final}.{gif,png}`）用 **sim_gt** 生成，确定性可复现。
- 解析 IK site 精度：collision-free 目标**中位 0.09mm**（亚毫米）；误差全来自与桌面/物体的物理碰撞，不是 IK。
- **回归**：Menagerie 默认路径不动——sim_gt pick 134mm / place_into 7mm，YOLO pick 113mm（检出 `[0.397,-0.115]` 1702 ROI，与历史一致）；`tests/test_bridge.py` 4/4 通过。

### ⚠️ 已知边缘项：YOLO 下 clear_table 的 +y bottle 抓取不稳

- **现象**：sim_gt（用 body 真值中心）bottle 稳定 173mm；YOLO 检出中心有 ~3mm 前向偏差（`[0.367,0.12]` vs 真值 `[0.37,0.12]`），到 +y bottle 上就把「pre-grasp 摆入路径」推过临界——机械臂摆到 pre 位姿时长指探到 bottle 的 +y 侧把它顶偏（0.12→0.07），随后只蹭起 ~26mm（<30mm 阈值）。cup 在 −y 侧对同量级误差不敏感（每次 127mm）。
- **根因不是 IK / 不是抓取参数**：解析 IK 精确、grip 力/摩擦/高度都试过；sim_gt 同位置同 grip 稳过。是 **+y 侧「无避障直达 pre-grasp」的摆入路径 + 感知 3mm 误差**共同踩中的运动学临界（−y 无此问题）。检测本身确定（bottle 中心逐次一致）。
- **后续修**（属运动规划/感知精修，非本次 IK 统一范围）：pre-grasp 摆入加简单避障 / 从物体上方竖直下降段接近（top-down 段）/ 再压低 wrist_cam 感知误差；或把可抓物体布局在 −y 稳定区。**当前交付以 sim_gt 为确定性验证路径。**

### 用法
```bash
# 解析 IK + piper_real 模型（--model real）；默认仍是 menagerie + 数值 IK
MUJOCO_GL=egl python3 -m multitask.executor --task clear_table --labels cup,bottle --container bowl --model real
MUJOCO_GL=egl python3 -m multitask.executor --task pick --target cup --model real --perception sim_gt
```
配置见 `multitask/config.yaml` 的 `model:` 段（menagerie / real：scene + analytic_ik + scan_poses + per-model grasp/detector 覆盖）。

### 下一步
- 真机部署：`piper_arm.solve_ik` 现在仿真真机同一套，`bridge.py` 的 `real_piper` 后端可接这套 IK；对齐 OpenHarmony 的 task 名/字段。
- run_nl / bridge 目前默认 menagerie，可加 `--model` 透传到它们。

## 2026-06-30 — IK 统一：解析 IK 改进 + URDF 模型验证（交接：待新模型上重搭）

**目标**：让仿真和真机用**同一套 IK**（runhanw 的 DH 解析 IK）。

**已完成 ✅**：
1. **解析 IK 改进**（`piper_arm.py:solve_ik`，保留原 `inverse_kinematics` 不动）：原版只算单分支→常返回无解（"挪一点就能解"）。改成枚举全分支（shoulder×elbow×wrist）+ FK 校验 + DH 雅可比数值兜底。**求解率 79%→89%→97%**（500 随机可达构型）。真机可直接用。
2. **根因 + 模型验证**：仿真里解析 IK 不收敛的 ~10mm，**是因为仿真用了 Menagerie（当初缺 mesh 的替代品），不是 DH 问题**。实测 runhanw 的 DH == repo 的 `config/piper_description.urdf`（<0.1mm 恒定偏移）。从 [AgileX 官方](https://github.com/agilexrobotics/piper_ros)（noetic `src/piper_description/meshes`）下了标准 STL 到 `sim/models/piper_real/meshes/`，建了 `sim/models/piper_real/piper_real.urdf`，**实测 DH-FK 与该模型差 <0.1mm**。→ 用这个模型，解析 IK 能精确驱动仿真。

**待做（下一步，需在新模型上重搭）**：见本条第 8 节 D7 「IK 改进」注脚 + 下方交接。简言之：在 `piper_real` 模型上补 actuator/夹爪 equality/home keyframe/TCP site，并入桌子+YCB 物体+相机，重调 wrist_cam 挂载 + 观察位姿 + 抓取参数，接 `arm.solve_ik` 取代 MuJoCo 数值 IK，复跑抓取验证。

**未改动仿真管线**（现有 multitask 抓取仍用 Menagerie + 数值 IK，pick 回归 113mm）。

## 2026-06-30 — 同步 runhanw：NL 解析 + bridge-ready 接口 + sim_gt 后端（已并入 main）

### main 同步（pull 到了）
- `git pull` 拉到 runhanw 的 3 个提交：`56839c5` NL demo、`dc4108d` bridge entrypoint、`3031838` pipeline setup。他把上次说的**关键词提取 + bridge 接口层**都 push 了。
- 我先前自写的 `multitask/command_parser.py` 只是等他版本时的临时替身，**功能被他的 `nl.py` 覆盖且他的更全（正则归一化、xyz 解析、容器别名、`ParsedTask` dataclass），已删除我那个**，统一用他的。

### runhanw 加的三层（在我们 multitask/ 之上）
1. **`nl.py` 规则 NL 解析**：中英文短命令 → `ParsedTask{task, source_label, container_label, place_xyz, labels, policy}` → 现有 executor（`execute_parsed_task`）。支持 pick / place_at / place_into / clear_table；容器别名 bucket/bin/box/桶/箱子 → bowl；`NLParseError` 干净报错。CLI `run_nl.py`：`python -m multitask.run_nl "把杯子放到碗里"`（`--dry-run` 只解析、`--perception sim_gt`）。
2. **`bridge.py` bridge-ready JSON 接口**（给上层 / OpenHarmony / ROS bridge 调 visual_grasp）：schema `visual_grasp.bridge.v1`；`normalize_command` 校验 + 归一（task/backend/perception 别名）；`--dry-run` **不 import MuJoCo**（无 sim 的机器也能测接口契约）；`execute_command` 派发到 TaskExecutor。**backend 显式分 `sim_mujoco`（已实现）vs `real_piper`（明确留空占位，真机后端后接）**——上层 JSON 契约不变，下层真机后端后补。
3. **`sim_gt` 感知后端**（`object_registry` + config `detector.backend`）：直接读 MuJoCo body 真值当"检测"，给**没装 YOLO/ultralytics 的机器**跑全仿真；YOLO 仍是默认。lazy import，缺 ultralytics 时明确报错而非静默兜底。

### 验证（本机）
- `nl.parse_nl("把杯子放到碗里")` → `place_into` cup→bowl ✓
- `bridge --dry-run --task place_into --source cup --target bowl` → schema `visual_grasp.bridge.v1`、dispatch `TaskExecutor.place_into` ✓

### 下一步（与 runhanw 对话同步）
- **IK 玄学问题**：真机用 DH 解析 IK，但**很多位姿解不出、挪一点点就能解**（不稳，"很玄学"）。runhanw 打算让 GPT 分析。→ 对应我们 **D7 sim2real 注脚**：解析 IK 非 100%（随机构型 round-trip ~78%），"挪一点就能解"正是腕部奇异 / Pieper 分解的边界 case；建议边界时留兜底（退数值 IK / MoveIt）。
- 完善 library；把 sim 做的搬到真机；**library / bridge JSON schema 和 OpenHarmony 那边对接**（对齐 task 名 + 字段）。

## 2026-06-29 — 多任务 library 开工：M1 物体注册表 + M2 pick（FSM）

### 0. 目标
按 [`TODO_multitask_yolo_grasp.md`](TODO_multitask_yolo_grasp.md) 起第一刀：**M1 物体注册表 + M2 pick 原语**，用**有限状态机执行器**串起来，先证明「包装层不破坏现有抓取」（Stage 1 单物体回归）。决策：起手范围 = M1+M2；executor 一上来就用 FSM（用户拍板）。

### 1. 新增 `multitask/` 包（包装现有 `sim/`，不重写）
- `config.yaml` / `config.py`：物体类别、容器、抓取/放置参数集中配置（避免硬编码）。注：COCO 能检的容器只有 `bowl`/`vase`，**没有 "box"**。
- `world.py`：`SimWorld` 轻量上下文（model/data/renderer/ctrl + settle/observe/set_arm/gripper），全建在 `sim/grasp_sim.settle` 等之上。
- `object_registry.py`：`detect_objects()` 把 `phase6_route_b.camera_object_point`（只返回第一个 cup）泛化成**所有检测实例**，每物体一条记录（label/conf/bbox/center_camera/center_base/n_roi/graspable/container）；`get_object(label, policy)` 支持 nearest / highest_confidence / leftmost / rightmost。复用 `backproject_roi` + `median_cluster` + 近表面半径修正。
- `primitives.py`：`plan_horizontal_grasp` / `move_to` / `open_gripper` / `close_gripper` / `execute_grasp` / `pick_object` / `release_at`，每个返回结构化 `{success,message,data}`。
- `executor.py`：`TaskExecutor` 有限状态机 `DETECT_SOURCE→PLAN_PICK→EXECUTE_PICK→VERIFY_GRASP→DONE/FAILED`，输出 JSON-safe 逐状态日志（顺带把 M5 的结构化接口先立起来）。CLI：`python -m multitask.executor --target cup`。
- `task_library.py`：`pick(label)` 薄封装。

### 2. 验证（Stage 1 单物体回归，headless `MUJOCO_GL=egl`）
- `MUJOCO_GL=egl python3 -m multitask.executor --target cup`：DETECT（cup_0 conf 0.39，base `[0.397,-0.115,0.168]`，1702 ROI）→ PLAN → EXECUTE（IK 全收敛）→ VERIFY → **DONE，cup 抬升 134mm ✅**。
- 证据：`multitask/evidence/pick_cup_final.png`（夹爪举杯）+ `pick_cup.gif`。
- **去 ground-truth**：抓取 z 改用「相机估计 + 桌面高度 clamp」（`table_top_z + clearance`），不再像 `grasp_sim_camera` 那样用 cup 真值 z；目标点全程来自相机（GT 只在 VERIFY 量抬升用）。

### 3. 踩坑
- FSM 的 VERIFY_GRASP 第一次就抓到真 bug：pre-grasp 若带水平姿态约束去接近，机械臂会把杯子**撞倒**（rose **-20mm**）。改成**位置-only 接近、只在 grasp/lift 锁水平**（对齐已验证的 `grasp_sim_camera` 序列）→ +134mm。说明 VERIFY 状态有用，没白加。

### 4. M2 收尾 / 其它
- M2 收尾：retry 逻辑（目前只有结构化错误消息，无重试）。

### 5. M3 place 任务（`place_at` ✅；`place_into`/`clear_table` 代码完整，但容器视觉检测确认不可行）

**已交付并验证（headless `MUJOCO_GL=egl`）**：
- `place_at(label, xyz)`：vision pick + 放到指定 base 坐标。FSM 加 `PLAN_PLACE→EXECUTE_PLACE→VERIFY_DONE` 三态。验证：cup 落点距目标 **62mm**（<80mm 容差），全绿。证据 `multitask/evidence/place_at_cup_final.png`。
- `pick(cup)` 在含 bowl 的多物体场景仍 **抬升 113mm**（多位姿扫描生效）。

**多位姿扫描（按用户选定方案实现）**：
- `object_registry.scan()`：依次走 config 里多个观察位姿，各自单帧检测后按 base-xy 邻近度 merge（center_base 与位姿无关，可去重）。解决了「cup 和容器同帧互相抑制」——bowl 在场景里时，cup 位姿 `[0.2,0.4,…]` 仍稳定检出 cup **0.45**（bowl 不在该帧）。
- 新建 `sim/models/piper/scene_multitask.xml`（嵌套 `include scene_grasp.xml` + bowl），**不动已验证的 `scene_grasp.xml`**，单物体抓取脚本不受影响。
- 加**深度门** `MIN_OBJECT_DEPTH=0.20`：拒绝近前景的夹爪误检（夹爪常被 YOLO 认成 airplane/toilet）。
- `perception.detect()` 增 `labels` 参数（向后兼容，默认仍 bottle/cup），让注册表能传更宽类别集。

**⚠️ 容器视觉检测 = 确认不可行（已穷举验证）**：
- 合成 bowl **YOLO 根本不认**：深度门 + 投影校验下扫了 **3 位置 × 36 位姿 = 108 组，0 次**把真 bowl 认成任何容器类。和 bottle 检不出是同一堵墙（模型级域差）。
- 容器类只吃到**误检**：夹爪→"toilet"（近，被深度门挡掉）、**cup→"vase"**（桌面深度，门挡不住，会被当成假容器放回 cup 自己位置）。所以 `containers` 收回成 `[bowl]`（不放 toilet/vase 这些误检磁铁），vision `place_into` 诚实报 `no container detected`。
- 物理层第二堵墙：即便给已知坐标，bowl 要避开 cup 帧（不压制 cup）就得放 `[0.50,0.18]`，那里**够不太到 + 长距离搬运中杯子从水平夹爪滑出**（落点偏 ~700mm）。

**阶段结论（被下面第 7 节突破推翻）**：**未贴图**的合成容器 YOLO 认不出。解决办法不是调形状/位姿，而是**用带贴图的真实 mesh**。

### 7. 容器突破：换成带贴图的真实 bowl mesh（YCB 024_bowl）→ place_into 全链路通（用户选定方案②）

用户判断「bowl 模型没建好」方向对，但根因是**贴图**不是形状：我先程序化生成了形状正确的半球 bowl mesh，YOLO 仍认成 cup/clock/frisbee（~40 组全 0 bowl）。换成 **YCB `024_bowl`（真实照片扫描红陶瓷碗，4K 贴图，research-free）**后，YOLO 在低角度（贴近腕部相机视角）稳定检出 **bowl 0.56–0.91**。**贴图是跨过域差的关键**。

- 资产：`sim/models/piper/assets/bowl_ycb.obj` + `bowl_ycb.png`；`scene_multitask.xml` 用它做容器（视觉 mesh 不参与碰撞 + floor/rim 盒子组成可接杯的 basin）。
- 容器定位改 **ray-to-table-plane**（bbox 中心射线打到 `table_z+0.02` 平面），比深度反投影对大凹物体稳（误差 ~32mm，bowl 宽 16cm，杯子落得进去）；graspable 仍用深度反投影。
- `place_into` 搬运用 **插值 waypoint**（`primitives.carry_to`）：一次性大跳会把杯子甩飞（实测 cup 飞到 x=1.39），分 6 段小步搬就稳。
- cup 选择加置信度 tie-break（8cm bucket），避免 bottle 偶发误检成 cup（0.38）被「最近」盖过真 cup（0.63）。

**M3 四个任务现全部 vision 跑通（headless `MUJOCO_GL=egl`）**：

| 任务 | 结果 |
|---|---|
| `pick(cup)` | 抬升 112mm ✅ |
| `place_at(cup,xyz)` | 落点 25mm ✅ |
| `place_into(cup)` | 检出 bowl **0.91** → 杯子放进碗，34mm ✅（证据 `multitask/evidence/place_into_cup_final.png`）|
| `clear_table([cup])` | 51mm ✅ |

`scene_grasp.xml` 保持干净（无 bowl），单物体抓取脚本不受影响。

### 8. M2 retry（✅ 已加）+ 下一步
- **M2 抓取重试已加**：FSM 回边 `VERIFY_GRASP` 失败→回 `EXECUTE_PICK` 重试，受 `grasp.max_retries`（默认 2）限制。验证：正常 pick 不触发重试；人为把 lift 阈值调到不可达时，EXECUTE_PICK 跑 3 次（1+2）后有界失败。
- 下一步：Stage 2 真·两物体（需第二个可靠检出的 graspable）——**bottle 检不出可照搬「YCB 贴图 mesh」解决**（换 YCB bottle 等带贴图模型）。

### 9. bottle 检出 + Stage 2 两物体（2026-06-30，同 YCB 贴图套路）

bottle 域差和 bowl 一样解决：换 **YCB `006_mustard_bottle`（贴图）**，缩放 0.6（窄轴 40mm，夹得动），YOLO 稳定检出 **bottle 0.79–0.89**（之前绿圆柱全 0）。再次印证「贴图 > 形状」。

- 资产 `assets/bottle_ycb.{obj,png}`；放进 `scene_multitask.xml` 做**可抓 free body** `ycb_bottle`。
- **名字冲突**：include 进来的占位绿瓶也叫 `bottle`，所以 YCB 瓶身命名 `ycb_bottle` + config `objects.body_of: {bottle: ycb_bottle}`（verify 的 label→body 映射）；`SimWorld.load` 把占位绿瓶挪到桌外藏起来（只剩真瓶，注册表无误检）。
- **抓取调参**：mesh 凸包夹取比 cup 弱，density 200→90 + friction 1.5→3 后稳抓（抬升 142–145mm）。

**Stage 2 / 多物体循环跑通**：`clear_table([cup, bottle], bowl)` → cup 检出 0.62 抓起 114mm 放进碗（45mm），bottle 检出 0.79 抓起 142mm 放进碗（11mm），全绿。证据 `multitask/evidence/clear_table_cup_final.png`（杯子+芥末瓶都在红碗里）。

**注册表三类物体现全部可靠 vision 检出**：cup（mug 几何）、bottle（YCB 贴图）、bowl（YCB 贴图）。

## 2026-06-29 —（队友 runhanw）wrist_cam viewer、目标过滤与更真实 cup 模型

> 来源：队友 **runhanw**（原 `visual_grasp` repo 作者；git 署名 `Valttery <ValtteryWang@gmail.com>`，即同一人）的 commit `636e2c7` "Add wrist-camera MuJoCo grasp viewer"。
> 他在 Phase 6（chuhan 完成的 Route B world_cam 真·视觉抓取）基础上，把**眼在手 `wrist_cam`（原计划的 Phase 7）**和实时 viewer 往前推了一大步，并新增多任务后续规划 [`TODO_multitask_yolo_grasp.md`](TODO_multitask_yolo_grasp.md)（459 行）。
> 下面 0–7 节为 runhanw 本人记录；第 8 节为他与 chuhan 对话同步出的待修项（chuhan 接手修）。

### 0. 本条目目标
- 让 MuJoCo 仿真能在 macOS viewer 中实时观察和调试；
- 将 Route B 从外置 `world_cam` 推进到腕部 `wrist_cam`；
- 修正腕部相机安装位姿、YOLO 误检、夹爪水平抓取和杯子视觉模型问题。

### 1. macOS viewer 调试入口
- 新增 `sim/grasp_sim_viewer.py`：用 `mujoco.viewer.launch_passive` 打开完整 MuJoCo 图形界面并实时执行抓取流程。
- macOS 上使用方式：
  - `MUJOCO_GL=glfw mjpython sim/grasp_sim_viewer.py`
  - `MUJOCO_GL=glfw mjpython sim/grasp_sim_viewer.py --fallback-ground-truth`
- viewer 默认保持打开：`--hold -1` 表示动作结束后一直等到用户关闭窗口。
- 支持初始 viewer 视角：
  - `--view wrist`：固定到腕部相机；
  - `--view world`：固定到外置相机；
  - `--view free`：自由相机。

### 2. wrist_cam 作为视觉抓取主相机
- `sim/phase6_route_b.py` 默认相机改为 `wrist_cam`。
- 新增腕部相机观察位姿 `WRIST_OBSERVE_CTRL`，让机械臂先退后并保持相对正的姿态，保证初始视野能覆盖桌面目标。
- `sim/grasp_sim_camera.py` 和 `sim/grasp_sim_viewer.py` 在识别前先进入观察位姿，再执行 YOLO + depth ROI 反投影。
- `sim/perception.py` 改为渲染 `wrist_cam` 并保存 `phase3_wrist_rgb.png` / `phase3_yolo_wrist.png`。

### 3. 腕部相机安装位姿修正
- 在 `sim/models/piper/piper.xml` 中将 `wrist_cam` 从夹爪侧面移到夹爪上方/后方。
- 修正相机 roll，使画面不再顺时针旋转 90 度。
- 当前相机设计目标：少遮挡夹爪、视线略微抬头、仍能看到桌面上的 cup。

### 4. YOLO 只识别任务目标类
- `sim/perception.py` 现在在 `model.predict` 阶段传入 `classes=[bottle, cup]`，只允许检测 `bottle` 和 `cup`。
- 保持原 pipeline 阈值 `conf=0.3`，没有通过降低阈值解决检测问题。
- 这样可以避免夹爪被 YOLO 误识别成 `airplane`、`toilet` 等无关类别。

### 5. 更真实的 cup 视觉模型
- `sim/models/piper/scene_grasp.xml` 中把原来的白色圆柱体升级为 mug-like cup：
  - 保留主圆柱作为真实 collision / mass body；
  - 增加 rim、opening、inner wall 和 handle 作为零接触视觉几何；
  - 目标是让 YOLO 看到更接近真实杯子的外观，而不是纯圆柱。
- 抓取物理仍由简单稳定的 cylinder collision 负责，避免视觉细节破坏抓取稳定性。

### 6. 水平夹爪抓取逻辑
- `sim/ik_mj.py` 新增 `target_horizontal_axis` 约束：不完全锁死末端姿态，只约束选定轴保持水平。
- `sim/grasp_sim.py` 新增：
  - `horizontal_grasp_frame(target_pos)`；
  - `horizontal_pregrasp_point(grasp_pos, grasp_mat)`。
- 抓取流程从“上方 pre-grasp”改为“水平后退 pre-grasp”，抓取和抬升阶段保持夹爪水平轴约束。

### 7. 当前已知事项
- `wrist_cam` 检测效果需要在本机 viewer 中继续用真实渲染画面验证；当前策略是改进物体外观和相机位姿，而不是降低 YOLO 阈值。
- 部分 MuJoCo 版本在包含 freejoint 物体后可能对 include 进来的 keyframe `qpos` 长度更严格；本地用户环境已能运行当前 scene，暂不改动 keyframe 结构。
- `sim/evidence/*` 中的截图/GIF 是调试产物，需要按最终版本重新生成后再作为正式证据提交。

### 8. 待修 bug 与改进方向（与 runhanw 对话同步，2026-06-29）

> 改到 `wrist_cam` 眼在手后冒出的两个问题，chuhan 接手修。以下为对话中确认的现象与排查方向。

1. **wrist_cam「YOLO 没找到 cup 但仍抓到」—— ✅ 已在本机 headless（`MUJOCO_GL=egl`）定位，结论如下：**
   - **喂给 YOLO 的相机没问题，确实是 `wrist_cam`**（`perception.py` `CAM="wrist_cam"`，`phase6_route_b.camera_object_point(cam="wrist_cam")`）。排除「传错相机」。
   - **真正原因：wrist_cam 观察位姿下 YOLO 啥都检不到**。`MUJOCO_GL=egl python3 sim/perception.py` → `target detections (none)`、`ALL detections (none)`。看渲染图 `sim/evidence/phase3_wrist_rgb.png` 就明白：**夹爪本体占满前景**，cup（白马克杯，画面右侧）又小又被夹爪指节半遮挡；绿瓶虽全见但 bottle 本来就检不出（domain-gap D6）。
   - **「仍能抓」来自兜底**：`grasp_sim_viewer.py:146-153` 在检测返回 `None` 且带 `--fallback-ground-truth` 时，直接用 ground-truth cup 位姿替代继续抓 → 表面看「没检出却抓到」。runhanw 在 macOS viewer 调试时正是带了这个 flag。
   - 反观无兜底的脚本行为正确诚实：`grasp_sim_camera.py` → `no cup detected by camera; abort`。
   - **修复方向（不是改相机接线）**：让 wrist_cam 观察位姿能干净看到 cup —— 退得更远 / 抬头俯视 / 把相机挪到能越过夹爪看桌面 / 抓取前把臂移到不挡视线的纯观察位姿；让 cup 在画面里更大更完整。修好后用 `perception.py` 复跑确认 cup 能稳定检出，再回 `grasp_sim_camera.py` 验证全链路。
   - **日志改进**：`grasp_sim_viewer.py` 走兜底时（已有 `using ground-truth fallback target` 打印）建议把它升成显眼 WARNING，避免演示时误以为是「真·相机抓取」。

   附带发现的**次要 crash**：`sim/phase6_route_b.py` 在 YOLO 无检出时崩溃 —— `camera_object_point` 返回 `bbox=None`，但 `main()` 第 122 行直接 `for v in bbox` → `TypeError: 'NoneType' object is not iterable`。需在 main 里加 `if p_base is None: print("no cup detected"); return` 的保护。

2. **抓取姿态写死水平导致部分 IK 无解** —— `sim/ik_mj.py` 的 `target_horizontal_axis` 把夹爪某轴锁成水平（配合 `grasp_sim.py` 的水平后退 pre-grasp），部分目标位姿下「position + 水平约束」无可行解。
   - 待办：让抓取姿态可退化/自适应（水平解不出时放宽轴约束，或回退到上方 top-down 抓取），而不是恒定锁死水平；记录无解的目标位姿便于复现。

3. **wrist_cam 本机 headless 真渲染验证 —— ✅ 已做（2026-06-29，chuhan）**：在 AWS（`MUJOCO_GL=egl`）跑了 `perception.py` / `grasp_sim_camera.py` / `phase6_route_b.py`。结论：**当前 wrist_cam 观察位姿这条眼在手链路在本机走不通**——YOLO 检不到 cup（夹爪遮挡 + cup 太小），所以相机驱动抓取无法真识别，只能靠兜底。即 runhanw 在 macOS viewer 看到的「能抓」基本都是 `--fallback-ground-truth` 的功劳，不是真·相机抓取。下一步先修观察位姿让 cup 可检出（见 bug #1 修复方向），再谈眼在手抓起。证据：`sim/evidence/phase3_wrist_rgb.png` / `phase3_yolo_wrist.png`（零检出）。

### 9. 修复结果（2026-06-29，chuhan，全程 headless `MUJOCO_GL=egl` 验证）

**bug #1（wrist_cam 检不到 → 兜底假抓）—— ✅ 已修复并端到端验证：**
- 根因 = 观察位姿下夹爪挡住画面中心、cup 太小/被半遮。用 headless 位姿扫描找新观察位姿 —— **关键坑**：必须在**完全 settle（~900 步）后**评估，否则取到的是 mid-swing 的假结果（一开始用 450 步扫到「能检出」的位姿，settle 到位后其实检不出，白调一轮）。
- 新观察位姿 `WRIST_OBSERVE_CTRL = [0.2, 0.4, 0, 0, 0, 0, 0.035]`（抬腕 j2=0.4 + base yaw j1=0.2），把 cup 甩到画面右上、完全离开夹爪。同步改了 `sim/phase6_route_b.py` 和 `sim/perception.py` 两处定义（旧值 `[0, 0.30, 0, 0, -0.17, 0, 0.035]`）。
- 验证：`perception.py` → cup conf **0.38**（之前 none）；`phase6_route_b.py` → 相机定位 cup |err|=**18mm**（xy18, z3），1702 ROI 点；`grasp_sim_camera.py` → 全链路相机驱动 **抓起 cup 116mm，LIFTED ✅**。证据已按工作位姿重生成：`phase3_wrist_rgb.png` / `phase3_yolo_wrist.png` / `phase6_camera_grasp_final.png` / `.gif`。

**去掉兜底（用户要求：啥都没检出直接报错）—— ✅ 完成：**
- `sim/grasp_sim_viewer.py`：删掉 `--fallback-ground-truth` 参数与分支，检测返回 None 直接 `raise RuntimeError`（不再用 ground-truth 假装抓）。
- `sim/grasp_sim_camera.py`：`no cup detected` 由 `print+return` 改为 `raise RuntimeError`。
- `sim/phase6_route_b.py`：补 None 保护，修掉 §8 记的 `bbox=None` 时 `TypeError` 次要 crash，无检出时 `raise SystemExit` 并给排查指引。

**bug #2（水平抓取写死致 IK 偶发无解）—— ⚠️ 加了安全网，但 headless 未能复现失败：**
- 改 `sim/grasp_sim.py:ik_target`：水平轴约束解不出时自动退回 position-only 解（D7 已知 position-only 收敛好、自然姿势手指近水平），不再硬崩 `IK did not converge`，并打印 `[ik] ... falling back to position-only`。三个抓取脚本共用这个 `ik_target`，一处改全覆盖。
- 但扫了 252 个目标位姿（桌面→抬升高度、xy 全覆盖）水平 IK **全部收敛**，没复现到「偶发无解」。所以这是**防御性安全网**，不是对着可复现 case 修的。用户那边「有时解不出来」可能是更早代码状态、或观察位姿→抓取的特定起始构型；后续真遇到时安全网会兜住。

## 2026-06-27（队友同步）— Route B / 仿真手眼标定 提为必需项

> 来源：与队友 runhanw（原 `visual_grasp` repo 作者）的对话同步。

### A. 需求核对（队友/讨论中提到的事项）

| 队友提到 | 含义 | 状态 |
|---|---|---|
| 把 mujoco 接上 | 模型进仿真 | ✅ 已完成（M1） |
| 仿真接 YOLO 搞识别 | 仿真相机 → YOLO | ✅ 已完成（M2，cup） |
| 看纯仿真能做到什么程度 | 能力验证 | ✅ 完整闭环 + 抓起 132mm（M4） |
| 扩充几个动作 / task library | 多动作库 | ⏸️ 延后（队友明确"先不急"） |
| 实体机械臂 / sim2real | 真机部署 | ❓ 待定（队友未拍板，我不主动推进） |
| **手眼标定的仿真等价**（对齐相机↔夹爪位姿） | 相机外参 | ⚠️ **关键缺口，见 B** |
| **真·用相机做 3D 定位** | Route B（depth 反投影） | ❌ 未做（现在用 ground-truth） |

### B. 关键发现：当前抓取还不是"真·视觉抓取"

队友 runhanw 一针见血地追问："相机和夹爪位姿怎么校准的？""如果用的相机做的话不可能不要[标定]啊。"

**他是对的。** 当前 `sim/grasp_sim.py` 的 3D 目标点直接取 `data.xpos[cup]`（**ground-truth 物体位姿，Route A**）；YOLO 只用来"确认检测到 cup"，它的 bbox **并没有参与 3D 定位**。所以现在：
- 抓取其实是"已知坐标抓取 + YOLO 摆设"，**没有真正用相机算物体位置**；
- 正因为没用相机反投影，所以"不需要"相机外参——这恰好暴露了缺口。

> 结论修正：之前把 Phase 6 当"可选打磨"是错的。**要让它成为名副其实的 visual grasp（也为了 design expo 的手眼标定叙事），Route B 是必需项，不是可选。**

### C. 仿真"手眼标定" = 从模型/URDF 直接读相机外参

真机：用棋盘格标定，估计 `link6_T_camera`（相机相对末端的位姿），再
`base_T_object = base_T_link6 · link6_T_camera · camera_T_object`。

仿真：**外参是已知的，直接从模型读，不用估计**（队友说的"写个坐标转换""从 urdf 里拿"就是这个意思）：
- 眼在手（`wrist_cam`，相机挂 link6）：`link6_T_camera = inv(world_T_link6) · world_T_camera`，二者都能读。**这是真机手眼标定的直接对应物**。
- 眼看手（`world_cam`，固定外置）：`base_T_camera = inv(world_T_base) · world_T_camera`（Phase 4 已算出，见 6.5）。
- MuJoCo 里相机位姿就是 `data.cam_xpos/cam_xmat`（`sim/transforms.py:camera_pose` 已封装）。

### D. Design Expo 叙事（可直接讲）

> 真机里相机和夹爪的相对位姿要用棋盘格做手眼标定；仿真里这个外参是模型已知量，直接从 MJCF/URDF 读出来（`link6_T_camera`）。**标定环节之后的整条变换链（camera→link6→base）和真机完全一样**——仿真只是把"估计外参"换成"读取外参"，验证了 pipeline 的正确性，也为真机标定提供了 ground-truth 对照。

### E. 重定优先级（更新 Phase 6）
- **P0（必需）**：Phase 6 Route B（相机 depth 反投影出 3D 点）+ 仿真手眼标定（读相机外参，把 ground-truth 换成相机算出来的点）。
- **延后**：task library / 扩充动作（队友"先不急"）。
- **待定**：实体机械臂（等队友决定）。
- ⚠️ **给 runhanw 发 PR 时需说明**：感知（YOLO bottle/cup 过滤）确实沿用了他的 `realsense_yolo_pc_roi.py`，坐标变换思路对应 `grasp_action.py`；但 **仿真里 IK 换成了 MuJoCo 雅可比 position-only（D7）**，没用 `piper_arm.py` 的解析 IK（解析 IK 在仿真坐标系下对 grasp 位姿不收敛）。所以"用了你那套代码"对感知成立、对仿真 IK 不成立——避免 runhanw 看 PR 时困惑。**但注意（见 D7 sim2real 注脚）：他的解析 IK 在 DH 帧里能解出这些水平抓取位姿（4/4），真机仍可用他那套；数值 IK 只是仿真权宜。**

### F. 相机方案（已决定，D8）
- **Phase 6 用 `world_cam`（眼看手 / eye-to-hand）**：现成可用、外参 `base_T_camera` 已算（6.5），最快把 Route B 跑通。标定故事是 camera↔base。
- **`wrist_cam`（眼在手 / eye-in-hand）移到 Phase 7**：贴近真机（腕部相机 + `link6_T_camera`），手眼标定叙事最直接，但需先解"观察位姿"，留到后面做。

---

## 2026-06-27 — 启动迁移：环境搭建 + 开发计划

### 0. 本条目目标
- 盘点 repo 现状，确定迁移技术路线；
- 安装仿真依赖（mujoco + ultralytics）；
- 产出一份可执行的分阶段开发计划（本条目主体）。
- 注意：本阶段**只出计划、搭环境，不写实现代码**（按用户要求 "先不着急直接完成"）。

### 1. 现状盘点（实测）

| 项 | 状态 | 说明 |
|---|---|---|
| `config/piper_description.urdf/.xacro` | ✅ 存在 | mesh 用 `package://piper_description/meshes/*.STL` 引用 |
| STL mesh 文件 | ❌ **repo 内缺失** | 直接转该 URDF→MJCF 会因缺 mesh 失败 |
| `piper_arm.py` | ✅ 完整 | DH 参数 / FK / Pieper IK / 关节限位 / 手眼外参齐全，IK 输出 6 关节角 |
| `realsense_yolo_pc_roi.py` | ✅ 完整 | 感知主程序：RealSense→YOLO(bottle/cup,conf>0.3)→ROI 深度反投影→`/object_point` |
| `grasp_action.py` | ✅ 完整 | 订阅目标点→cam→link6→base 变换→IK→Piper SDK 控制 |
| `yolo11n.pt` | ✅ 存在 | YOLO 权重就绪 |
| `torch / numpy / scipy / matplotlib` | ✅ 已装 | numpy 2.2.6, torch 2.12.0, Python 3.10.12 |
| `mujoco` | ✅ 3.10.0 | 本次安装，headless 渲染已验证 |
| `ultralytics` | ✅ 8.4.80 | 本次安装，权重 `yolo11n.pt` 已就绪 |
| `rospy / pyrealsense2 / piper_sdk` | ❌ 未装 | 仿真路线**不需要**，刻意不装 |

### 2. 技术路线决策

**决策 D1：Piper 模型用 MuJoCo Menagerie 官方 `agilex_piper`，不手转 repo 内 URDF。**
- 理由：repo 的 URDF 缺 STL mesh；手动 URDF→MJCF 是计划文档列的首要风险（mesh/mimic/collision 不兼容）。Menagerie 的 `agilex_piper` 是**同一款 AgileX Piper**，自带调好的 MJCF + mesh + actuator，一步消掉「缺 mesh + 转换失败」两个坑。
- 校验方式：`piper_arm.py` 的 DH/IK 针对的就是真实 Piper，关节角应能对上。Phase 1 用 FK 对比（同一组关节角，DH-FK 末端位姿 vs MuJoCo body 位姿）验证一致性。

**决策 D2：仿真路线不引入 ROS。**
- 第一版做成独立 Python 脚本闭环（MuJoCo + numpy + ultralytics），不依赖 rospy。原 repo 的 ROS topic 仅作概念参考。后续若主项目要 ROS2，再封装为 service/bridge（计划文档 Milestone 5）。

**决策 D3：3D 目标点先走 Route A（ground-truth），再补 Route B（depth 反投影）。**
- 先用 MuJoCo `object body pose` 当目标点验证控制闭环（不被相机内参/深度误差阻塞），跑通后再恢复「YOLO bbox → depth ROI → 反投影」的完整视觉链路，复现真机 pipeline。

### 3. 分阶段开发计划

> 里程碑编号沿用计划文档第 14 节（Milestone 1~5）。

#### Phase 0 — 环境 & 模型就绪（✅ 完成）
- [x] 安装 `mujoco` 3.10.0 + `ultralytics` 8.4.80（torch 2.12.0+cu130 / numpy 2.2.6 / Python 3.10.12）
- [x] headless 渲染冒烟测试通过（trivial 模型渲染出 PNG）
- [x] 获取 Menagerie `agilex_piper` → `sim/models/piper/`（sparse-checkout，含 piper.xml / scene.xml / assets）
- [x] 用 piper MJCF headless 加载 + step + 渲染（`sim/check_model.py` → `sim/evidence/phase0_piper_home.png`）
- **产物**：依赖版本记录 ✅、模型结构打印 ✅、home 位姿渲染图 ✅

> **⚠️ 重要：headless 渲染环境**
> 本机是无显示器的 AWS 服务器（Tesla T4 GPU，无 X11），MuJoCo 默认用 GLFW 会报
> `gladLoadGL error / DISPLAY missing`。**所有渲染脚本必须设环境变量 `MUJOCO_GL=egl`**
> （走 GPU 离屏渲染，已验证可用）。退出时 PyOpenGL 会打印一条 `EGLError in __del__` 的
> 无害告警，可忽略。

#### Phase 1 — Piper in MuJoCo（✅ 完成 → Milestone 1）
- [x] headless 加载 piper MJCF，打印 joint / actuator 列表（Phase 0 已做）
- [x] 建立 **joint mapping table**：1:1 直映射，无符号翻转（见 6.2）
- [x] FK 一致性验证：姿态精确一致，位置 ~10mm RMS 残差（见 6.2）
- [x] 驱动关节运动（位置控制）+ 夹爪开合，方向正确
- **产物**：joint mapping 表 ✅、FK 对比日志 ✅、运动 montage ✅（`sim/evidence/phase1_motion.png`）
- **判定**：✅ 仿真里能看到 Piper，关节按预期动，DH 与 MuJoCo 末端**姿态一致、位置 ~10mm**

#### Phase 2 — 抓取场景 & 相机（✅ 完成）
- [x] 在 MJCF 里加 table + bottle + cup（绿瓶+白杯，free body，已稳定 settle）
- [x] 加相机：`wrist_cam`（装 link6）+ `world_cam`（fixed）
- [x] 渲染 RGB 截图确认视角合理（world_cam 两物体清晰）
- **产物**：`sim/models/piper/scene_grasp.xml`、`sim/evidence/phase2_world_cam.png` / `phase2_wrist_cam.png`（详见 6.3）

#### Phase 3 — MuJoCo 相机 → YOLO（✅ 完成 → Milestone 2，cup）
- [x] 从 `world_cam` 渲染 RGB，喂给 YOLO（`sim/perception.py`，沿用 bottle/cup, conf>0.3 过滤）
- [x] YOLO 输出 bbox + confidence：cup conf=0.75
- [x] 扫描相机/颜色应对 domain gap：cup 稳定检出；bottle 原始几何全检不出（见 6.4）
- **产物**：YOLO 标注图 ✅、bbox+conf 日志 ✅、`detect()` 复用接口 ✅
- **判定**：✅ 仿真画面里的 **cup** 能被 YOLO 稳定检出（bottle domain-gap，D6 延后）

#### Phase 4 — 坐标变换（✅ 完成 → Milestone 3，Route A）
- [x] 读取 `world_T_base / world_T_camera / world_T_object`
- [x] 实现 `base_T_object = inv(world_T_base) · world_T_object`（`sim/transforms.py`）
- [x] 明确 MuJoCo camera(+y up,-z) vs ROS optical(+y down,+z) 轴向差异，打印矩阵
- [x] 可视化验证：GT 中心投影回图像，落在 YOLO bbox 内
- **产物**：各 frame 矩阵 ✅、`base_T_cup=[0.38,-0.11,0.17]` ✅、投影验证图 ✅
- **判定**：✅ 目标点转到 base frame 后与场景一致（TRANSFORM VERIFIED）

#### Phase 5 — 仿真抓取闭环（✅ 完成 → Milestone 4，**抓起 132mm**）
- [x] 物体改可夹尺寸（cup 直径 52mm < 70mm 开口）+ 减重 ~50g + 加摩擦
- [x] `base_T_cup` → MuJoCo 雅可比 position-only IK（D7）→ 关节角
- [x] pre-grasp → 下降 → 闭合 → 抬升，ctrl 位置伺服驱动
- [x] 录完整 attempt（montage + GIF）
- **产物**：`sim/grasp_sim.py`/`sim/ik_mj.py`、final/montage/GIF、cup 抬升 132mm 日志
- **判定**：✅ 完成一次 **cup 抓起并抬升 132mm**（可复现）

#### Phase 6 — 完整视觉链路 & 仿真手眼标定（**必需**，用 `world_cam` 眼看手，→ Milestone 5）
> 队友同步后从"可选"提为"必需"：没有它，抓取就不是真·视觉抓取（见上方队友同步 B 节）。相机方案 = `world_cam`（D8）。
- [x] **仿真手眼标定**：从模型读 `world_cam` 外参 `base_T_camera`，文档化为真机手眼标定的对应物（C/D 节、6.7）
- [x] 读 MuJoCo `world_cam` depth，按 YOLO cup bbox ROI 反投影 → camera 系点（+ bbox 半径修正近表面偏差）
- [x] 把 ground-truth（Route A）换成相机算出的点（Route B）→ base 系 → 复跑抓取仍能抓起（138mm，误差 7mm）
- [ ] 封装可被外部（未来 OpenHarmony command bridge）调用的 grasp action 入口（command format + I/O 日志）——剩余
- **产物**：✅ 相机外参/标定说明、depth 反投影、Route-B 抓取证据（6.7）；⬜ action API
- **判定**：✅ 3D 目标点来自相机（非 ground-truth），成功抓起 cup（138mm）

#### Phase 7 — wrist_cam 眼在手 + 多任务规划（🚧 进行中，队友 runhanw，2026-06-29，commit `636e2c7`）
- **`wrist_cam` 眼在手（eye-in-hand）**：腕部相机 + `link6_T_camera`，最贴近真机手眼标定（D8 从 Phase 6 移来）。
  - ✅ runhanw 已做：`sim/phase6_route_b.py` 默认相机改 `wrist_cam` + 新增观察位姿 `WRIST_OBSERVE_CTRL`（先后退保持相对水平，让初始视野盖住桌面目标）并打印 `link6_T_camera`；修正 `wrist_cam` 安装位姿（夹爪侧面→上方/后方、修 roll 不再转 90°）；`sim/grasp_sim_viewer.py` macOS 实时 viewer（`mjpython` + `launch_passive`，`--view wrist/world/free`、`--hold -1`、`--fallback-ground-truth`）；`sim/perception.py` YOLO `classes=[bottle,cup]` 限类（根治夹爪被误检成 airplane/toilet，阈值仍 0.3 不降）；`scene_grasp.xml` 升级 mug-like cup（主圆柱仍做碰撞/质量，rim/开口/内壁/把手为零接触视觉几何）；`sim/ik_mj.py` 加 `target_horizontal_axis` + `grasp_sim.py` 水平后退抓取。
  - ⚠️ 未完成：本机 headless 真渲染验证；两个待修 bug（见上方 2026-06-29 条目第 8 节）。
- **多任务 task library（原 Phase 7+ / Plan 第 12.5 节 P4）**：runhanw 写出 [`TODO_multitask_yolo_grasp.md`](TODO_multitask_yolo_grasp.md)——object registry（多检测实例）+ grasp/place 原语 + task library（`pick`/`place_at`/`place_on`/`place_into`/`clear_table`）+ FSM executor；明确先不做 NL/LLM/VLA，只做规则化、可测试的执行层并最大化复用现有感知 + IK。**尚未开工**。
- LLM JSON command / VLA / 真机部署 —— 仍延后（避免 scope creep）。

### 4. 目录结构规划（待 Phase 1 落地，本条目尚未创建）

```text
visual_grasp/
├── sim/                      # 新增：仿真迁移代码
│   ├── models/
│   │   ├── piper/            # Menagerie agilex_piper (MJCF + meshes)
│   │   └── scene_grasp.xml   # table + bottle + cup + camera + piper
│   ├── mujoco_camera.py      # 从命名相机渲染 RGB + depth
│   ├── perception.py         # YOLO on MuJoCo RGB（移植自 realsense_yolo_pc_roi.py）
│   ├── transforms.py         # world/base/cam/object frame 变换（含轴向修正）
│   ├── piper_mj.py           # piper_arm.py IK 关节角 ↔ MuJoCo qpos/ctrl 映射
│   ├── grasp_sim.py          # 主闭环：检测→3D点→base→IK→移动→夹爪
│   └── evidence/             # 截图 / 日志 / 视频
├── piper_arm.py              # 复用（DH/IK，不改或最小改）
└── ...（原 repo 文件保留）
```

### 5. 关键风险与对策（沿用计划文档第 16 节，落到本路线）

| 风险 | 对策 |
|---|---|
| joint 顺序/方向/offset 不匹配 | Phase 1 建 joint mapping table + FK 对比，先验证再用 IK |
| MuJoCo camera frame ≠ ROS optical frame | Phase 4 显式写清各轴向，打印矩阵，marker 验证 |
| YOLO 不识别仿真物体（domain gap） | 真实贴图 / COCO 风格物体 / 调光照与相机角度 |
| 夹爪 mimic/contact 不稳 | 第一版只做闭合动作，后续再调 contact/friction |
| numpy 2.x 与某些库不兼容 | 出问题时锁版本（记录在对应条目） |

### 6. 进度跟踪（总表）

| Phase | 里程碑 | 状态 |
|---|---|---|
| Phase 0 环境&模型 | — | ✅ 完成 |
| Phase 1 Piper in MuJoCo | M1 | ✅ 完成 |
| Phase 2 场景&相机 | — | ✅ 完成 |
| Phase 3 相机→YOLO | M2 | ✅ 完成（cup） |
| Phase 4 坐标变换 | M3 | ✅ 完成 |
| Phase 5 抓取闭环 | M4 | ✅ 完成（抓起132mm） |
| Phase 6 真·视觉抓取+手眼标定 | M5 | ✅ 核心完成（相机驱动抓起138mm）；action API 剩余 |
| Phase 7 wrist_cam眼在手 + 多任务规划 | — | 🚧 进行中：wrist_cam 眼在手已 **headless 验证通过**（chuhan 修好观察位姿，相机驱动抓起 116mm）；bug #1 + 兜底 + 次要 crash 已修；bug #2 加了 IK 安全网（未复现失败）。多任务 `TODO_multitask_yolo_grasp.md` 仍未开工 |

### 6.1 Phase 0 执行结果（2026-06-27）

**模型来源**：MuJoCo Menagerie `agilex_piper`（同源于 `agilexrobotics/Piper_ros` 的 URDF，与 `piper_arm.py` 同一款机器人）。已 sparse-checkout 到 `sim/models/piper/`，自带 MJCF + mesh + position actuator + gripper mimic 约束。

**模型结构**（`nq=8 nv=8 nu=7 nbody=10`）：
- 关节：`joint1..joint6`（hinge，机械臂）+ `joint7`/`joint8`（slide，夹爪两指）。`joint8` 通过 equality 约束镜像 `joint7`。
- 执行器：`joint1..joint6` + `gripper`（驱动 `joint7`）。即 **ctrl 7 维**：`[j1..j6, gripper]`；**qpos 8 维**：`[j1..j6, j7, j8]`。
- `home` keyframe：`qpos=[0, 1.57, -1.349, 0,0,0, 0,0]`，`ctrl=[0,1.57,-1.349,0,0,0,0]`。
- home 位姿（settle 后）body 世界坐标：`base_link=[0,0,0]`、`link6=[0.370,0.001,0.383]`、`link7≈link8=[0.504,0.001,0.365]`。

**关节限位对照**（MJCF rad→deg vs `piper_arm.py` `link_limits`）—— 同一机器人，MJCF 略保守，Phase 1 IK 可行性检查需注意差异：

| 关节 | MJCF (deg) | piper_arm.py (deg) | 备注 |
|---|---|---|---|
| j1 | -150.0 ~ 150.0 | -154 ~ 154 | 接近 |
| j2 | 0 ~ 179.9 | 0 ~ 195 | MJCF 上限更小 |
| j3 | -154.5 ~ 0 | -175 ~ 0 | MJCF 下限更小 |
| j4 | -104.9 ~ 104.9 | -106 ~ 106 | 接近 |
| j5 | -69.9 ~ 69.9 | -75 ~ 75 | MJCF 略小 |
| j6 | -179.9 ~ 179.9 | -100 ~ 100 | MJCF 更宽 |

**关键工程注意（Phase 1 用）**：
- MJCF 渲染默认离屏 framebuffer 仅 640×480；要更大分辨率须先设 `model.vis.global_.offwidth/offheight`（已在 `check_model.py` 处理）。
- 夹爪：写 `ctrl[gripper]`（0=闭合 ~ 0.035=张开），`joint8` 自动镜像，不要直接写 `joint8`。
- Phase 1 FK 对齐时，DH 的 base/link6 帧约定与 MJCF body 帧约定可能差一个常量旋转——先比对 link6 原点**位置**，姿态差异单独标定。

**证据**：`sim/check_model.py`（可复跑）、`sim/evidence/phase0_piper_home.png`。

### 6.2 Phase 1 执行结果（2026-06-27）→ Milestone 1 达成

**Joint mapping table**（`piper_arm.py` ↔ MJCF，经 FK 暴力搜索验证）：

| piper_arm.py | MJCF joint | MJCF actuator | ctrl idx | qpos idx | 符号 |
|---|---|---|---|---|---|
| j1 | joint1 | joint1 | 0 | 0 | + |
| j2 | joint2 | joint2 | 1 | 1 | + |
| j3 | joint3 | joint3 | 2 | 2 | + |
| j4 | joint4 | joint4 | 3 | 3 | + |
| j5 | joint5 | joint5 | 4 | 4 | + |
| j6 | joint6 | joint6 | 5 | 5 | + |
| gripper | joint7 (+joint8 mimic) | gripper | 6 | 6,7 | 0=闭合, 0.035=单指张开 |

> 关键结论：**关节 1:1 对应，无符号翻转，无需重排**。`piper_arm.py` 的关节输入可直接写入 MuJoCo `qpos[0:6]` / `ctrl[0:6]`。

**FK 对齐验证**（`sim/phase1_fk_check.py`，6 组测试位姿）：
- **姿态完全一致**：`R_mj^T @ R_dh ≈ I`，跨所有位姿最大偏差 0.0001（Frobenius）→ DH 链与 MJCF 旋转结构完全相同。
- **位置残差 ≈ 10mm RMS**：暴力搜索 64 种符号组合，最优即全 `+1`，平均误差 9.96mm，且各位姿都稳定在 ~10mm。
- **残差来源**：把误差投影到 link6 局部系，offset 非恒定（std ~2.5mm）→ 不是帧定义差，而是 `piper_arm.py` 手调 DH 连杆参数（`a`/`d`）相对真实 MJCF 几何略有近似。
- **判定**：对"移动到中心点+闭合夹爪"的抓取 baseline（夹爪单指行程 35mm）**可接受**。若后续需要更高精度，两条路：① 用 MJCF 几何反推/微调 DH 的 `a`/`d`；② 仿真里直接用 MuJoCo 数值 IK（jacobian/mink）替代 DH-IK。

**运动 & 夹爪验证**（`sim/phase1_motion.py`）：4 个位姿（home / reach-open / reach-closed / twist-closed）位置伺服收敛正常；夹爪 `ctrl[6]` 张开测得总开口 70mm（两指各 35mm，equality mimic 生效）、闭合 0mm。

**证据**：`sim/phase1_fk_check.py`、`sim/phase1_motion.py`、`sim/evidence/phase1_motion.png`。

### 6.3 Phase 2 执行结果（2026-06-27）

**场景文件**：`sim/models/piper/scene_grasp.xml`（放在 `piper.xml` 同级目录，保证 `meshdir="assets"` 解析正确——与计划文档原写的 `sim/models/scene_grasp.xml` 路径不同）。`include piper.xml` + 自带 floor/skybox/light。

**场景内容**（`nq=22 nbody=13 ncam=2`）：
- table：低桌，桌面 z=0.12；位于机械臂前方 x=0.42。
- bottle：绿色圆柱 + 细颈（free body），settle 后稳定在 `[0.38, 0.11, 0.18]`。
- cup：白色矮粗圆柱（free body），settle 后稳定在 `[0.38, -0.11, 0.17]`。
- 两个相机：`world_cam`（固定，targetbody 对准 table）+ `wrist_cam`（眼在手，装 link6）。
- 注：物体是 free body；keyframe `home` 仅 8 维 qpos 但 MuJoCo 3.10 容忍短 keyframe（脚本里用 `ctrl` 设臂姿、靠重力 settle 物体）。

**相机位姿**（settle 后）：
- `world_cam` pos=`[0.45,-0.65,0.60]`，forward(-z)=`[-0.03, 0.73,-0.68]`（俯视桌面，两物体清晰无遮挡）。
- `wrist_cam`（look pose 时）pos≈`[0.30,-0.11,0.39]`，forward=`[0.95,0.29,-0.12]`；从 link6 后/侧方挂载（`pos="0 -0.11 -0.07"`）避开夹爪遮挡。

**决策 D5**：**baseline 主感知相机用固定 `world_cam`**（视野干净、外参静态、坐标变换最简、最快闭环）。`wrist_cam` 已能看到夹爪之外场景，作为更贴近真机的眼在手方案保留，但不阻塞 Phase 3-5。

**证据**：`sim/phase2_scene.py`、`sim/evidence/phase2_world_cam.png`（场景总览）、`sim/evidence/phase2_wrist_cam.png`（眼在手视角）。

### 6.4 Phase 3 执行结果（2026-06-27）→ Milestone 2 达成（cup）

**实现**：`sim/perception.py`——从 `world_cam` 渲染 RGB（640×480）→ `yolo11n.pt`，沿用真机 pipeline 的类别过滤（bottle/cup，conf>0.3）。提供可复用的 `detect(rgb)`，返回 target_boxes + 全量检测 + 标注图，供 Phase 4-5 调用。

**结果**：
- **cup 稳定检出 conf=0.75**，bbox=`[246.7, 145.3, 319.1, 250.0]`。✅
- **bottle（原始圆柱+颈）检不出（0.00）**——不是颜色问题。
- airplane 误检（arm 部件，conf~0.4-0.47）：非目标类，已被过滤，不影响 pipeline。

**domain-gap 扫描**（3 相机 × 5 瓶子颜色，共 15 组）：
- cup 在所有组合都检出（0.40–0.78），top-ish 相机最佳（~0.77）。
- bottle 在所有组合**全 0.00**（green/dkgreen-glass/clear/red/amber 都不行）。
- 结论：原始几何圆柱无论颜色/角度都不被 COCO-YOLO 当作 "bottle"；矮粗圆柱反而能稳定被认成 "cup"。

**决策 D6**：**baseline 用 cup 作为可靠抓取目标**（detect→3D→IK→grasp 闭环对 cup/bottle 完全一样，cup 足以验证全链路）。bottle 的 YOLO domain-gap 留待后续用**真实 bottle mesh 或贴图**解决，不阻塞最小闭环。

**相机锁定**：`world_cam` pos=`[0.45,-0.60,0.55]` fovy=52（cup 检出最佳 + 俯视利于 depth→3D）。

**证据**：`sim/perception.py`、`sim/evidence/phase3_world_rgb.png`、`sim/evidence/phase3_yolo_world.png`（cup 0.75 标注）。

### 6.5 Phase 4 执行结果（2026-06-27）→ Milestone 3 达成

**实现**：`sim/transforms.py`（Route A，ground-truth）。可复用 `body_pose / camera_pose / cam_intrinsics / project`。

**变换结果**：
- `world_T_base` 在原点 + 单位旋转 → **base 系 == world 系**（base_link 是根 body，无 pos/quat）。
- `base_T_cup`：单位旋转 + 位置 `[0.38, -0.11, 0.17]`（= cup world 位置，符合预期）。
- `world_T_cam` = `[0.45,-0.60,0.55]`；`base_T_cam` 同（因 base==world）。
- `world_cam` 内参（fovy=52, 640×480）：fx=fy=492.1, cx=320, cy=240。

**帧约定（已写进脚本头注释，Route B 需要）**：
- MuJoCo camera frame（`cam_xmat` 列=相机轴）：**+x right, +y up, 看 -z**（OpenGL 约定）。
- ROS optical frame（真机 RealSense pipeline 假设）：**+x right, +y down, +z forward**。
- 转换：`ros_optical = diag(1,-1,-1) · mujoco_cam`。Route A 全程在 world/base 系，不受此影响；Phase 6 的 depth 反投影才需要。

**验证（一次性交叉验证 4 件事）**：把 ground-truth cup 中心投影回 `world_cam` 图像 → 像素 `(283.9, 198.4)`，depth 0.62m，**落在 YOLO cup bbox 内** → 同时验证了物体位姿、相机外参、相机内参、YOLO 检测四者一致。`VERDICT: TRANSFORM VERIFIED`。

**证据**：`sim/transforms.py`、`sim/evidence/phase4_target_proj.png`（红十字=GT 投影，黄框=YOLO bbox，重合在 cup 上）。

### 6.6 Phase 5 执行结果（2026-06-27）→ Milestone 4 达成（**真正抓起来**）

**目标**（按用户要求重定范围）：不止"靠近+闭合"，而是**真正把 cup 抓起并抬升**。

**先解决可夹性**（之前漏的关键问题）：
- 原 cup 直径 84mm > 夹爪最大开口 70mm，**根本合不拢**。已缩到直径 52mm（半径 0.026）。
- 原物体用默认水密度 → 0.55kg，太重。cup 设 `density=235` → **~50g**（真实量级）；`friction=1.5 0.1 0.01`（增大摩擦防滑）。
- 缩小后 YOLO 仍检出 cup（conf 0.43-0.46，>0.3 阈值），detect→grasp 故事保持。

**IK 路线（关键决策 D7）**：
- `piper_arm.py` 解析 IK（Pieper）**对所有 grasp 位姿都失败**（top-down/各 pitch 全 `no feasible solution`/`fail theta 3`）。
- 进一步发现：**top-down 在桌面任何位置都不可达**——这台小臂前伸够物时腕部无法竖直朝下（关节限位 j4≈105°、j5≈-70° 顶死）。
- 改用 **MuJoCo 雅可比数值 IK**（`sim/ik_mj.py`，阻尼最小二乘，TCP=site，仅控 6 个臂关节）——D4 已预留此后备。
- **再一个关键发现**：full-orientation IK 也难收敛，但 **position-only IK 完美收敛（1mm）**；且自然到位姿势下**手指闭合轴 ≈ 水平**（`[-0.28,-0.96,0.01]`，沿 base-y）→ 对竖直圆柱正好是水平夹取。**所以不控姿态，只控位置即可**。

**TCP 标定**：site 从 link6 z=0.16（在指尖之外）下移到 **z=0.10 = 指垫中点**，IK 才能把"指间"对准 cup（不是把指外的点对准）。

**抓取序列**（`sim/grasp_sim.py`，position-only IK + warm-start，ctrl 位置伺服驱动物理）：
`YOLO 确认 cup → Route A 取 base_T_cup → pre-grasp(上方,张开) → 下降到 grasp(张开) → 闭合(ctrl[6]→-0.005) → 抬升(re-IK 上移)`

**结果**：
- YOLO cup 检出 ✅；pre-grasp/grasp/lift IK 均收敛。
- 闭合后 `gripper_qpos=0.0257`（≈ cup 半径 26mm，正好夹在杯壁）。
- **cup 从 z=0.165 升到 0.297，抬升 132mm → `LIFTED` ✅**（可复现）。

**证据**：`sim/grasp_sim.py`、`sim/ik_mj.py`、`sim/evidence/phase5_final.png`（夹爪举着 cup）、`phase5_grasp_montage.png`、`phase5_grasp.gif`（全过程动图）。

> ✅ **至此，计划文档第 17 节的"最小可交付闭环"全部跑通**：MuJoCo Piper → 仿真相机 → YOLO 检测 → 3D 目标 → base 系变换 → 机械臂移动 → 夹爪闭合 → **抓起**。

### 6.7 Phase 6 执行结果（2026-06-27）→ 真·视觉抓取 + 仿真手眼标定

**做到了**：3D 目标点**完全来自相机**（不再用 ground-truth）。

**Route B 链路**（`sim/phase6_route_b.py` + `sim/perception.py:render_depth`）：
`world_cam RGB → YOLO cup bbox → world_cam depth → ROI 反投影到 camera 系点 → base_T_camera 转到 base 系`
- 反投影用 `sim/transforms.py:project` 的逆（camera 系：+x right,+y up,-z fwd）；MuJoCo depth 约定验证通过。
- **近表面偏差修正**：反投影点落在杯子朝相机的近表面，会偏向相机 ~半径。用 bbox 宽度 × 深度 / fx 估出半径（**纯相机信息，无 ground-truth**），把点沿视线推离相机一个半径 → 近似杯心。
- 精度：修正前误差 24mm（xy 15），**修正后 7mm（xy 7, z 2）**，落在夹取余量（单边 9mm）内。

**仿真手眼标定 = 从模型直接读外参**：`base_T_camera`（`data.cam_xpos/cam_xmat`，见 6.5/C 节），不用棋盘格估计。

**相机驱动抓取**（`sim/grasp_sim_camera.py`）：用相机算出的点跑整条 pre-grasp→下降→闭合→抬升 → **cup 抬升 138mm，LIFTED ✅**（对照 ground-truth 仅差 7mm）。

> 对比 Phase 5：Phase 5 用 ground-truth 坐标（Route A）；**Phase 6 这条是名副其实的 visual grasp**——目标点是相机看出来的，并且真正用上了相机外参（手眼标定）。

**证据**：`sim/phase6_route_b.py`、`sim/grasp_sim_camera.py`、`sim/evidence/phase6_camera_grasp_final.png`、`phase6_camera_grasp.gif`。

**Phase 6 剩余**：封装可被 OpenHarmony command bridge 调用的 grasp action 入口（command format + I/O 日志）——未做，OpenHarmony 集成时再补。

### 7. 决策记录
- **D1**：用 Menagerie `agilex_piper`，不手转 repo URDF。
- **D4**（Phase 1）：关节 1:1 直映射（无符号翻转）。DH-FK 姿态精确、位置 ~10mm 残差，baseline 可接受；高精度需求时再微调 DH 或改用 MuJoCo 数值 IK。
- **D2**：仿真第一版不引入 ROS，独立 Python 闭环。
- **D3**：3D 目标点先 Route A(ground-truth) 后 Route B(depth)。
- **D5**（Phase 2）：baseline 主感知相机用固定 `world_cam`；`wrist_cam` 作为眼在手备选保留。
- **D6**（Phase 3）：baseline 抓取目标用 cup（YOLO 稳定检出 0.75）；bottle 原始几何检不出，留待真实 mesh/贴图，不阻塞闭环。
- **D7**（Phase 5）：抓取用 **MuJoCo 雅可比 position-only IK**（`sim/ik_mj.py`），不用 `piper_arm.py` 解析 IK。原因：解析 IK 对 grasp 位姿全失败，且小臂在桌面无法竖直朝下；position-only 收敛好，自然姿势手指水平闭合恰好夹圆柱。`piper_arm.py` 的 DH/FK 仍是有效的真机参考（D4）。
  - **sim2real 注脚（2026-06-30 验证）**：上面「解析 IK 对 grasp 位姿全失败」是**仿真坐标系**下的结论（DH 几何 vs MuJoCo 模型差 ~10mm + 当初喂的目标姿态是 MuJoCo TCP 帧约定，没转成 DH 帧）。**重新在 DH 帧内验证：`piper_arm.py` 解析 IK 能解出我们实际用的水平侧夹位姿（cup/bottle 的 grasp+lift，4/4）**；随机有效构型 round-trip 235/300（~78%，有腕部奇异等边界 case）。
  - **结论**：**真机路线 = 用 runhanw 的解析 IK + 水平侧夹策略即可**，MuJoCo 数值 IK 只是仿真权宜；那 ~10mm 是仿真特有，真机 DH 对得齐不吃亏。**两个集成注意**：① 把目标姿态从 MuJoCo TCP 帧转到 DH 帧（差一个常量旋转，第一版没转→0/25 全崩）；② 解析 IK 非 100%，边界 case 留兜底（退数值 IK / MoveIt）。验证脚本逻辑见 scratchpad `test_analytic2.py`（FK→IK round-trip）。

  - **IK 改进（2026-06-30，回应 runhanw「很多位姿解不出、挪一点就能解」）**：
    - **根因 = 单分支**：原 `inverse_kinematics()` 只算一个分支（单 theta1、单肘）→ 该分支超限就返回 False，即使别的分支有解。"挪一点就能解"就是这个。
    - **改进 = 枚举全分支 + 数值兜底**，加进 `piper_arm.py:solve_ik()`（保留原 `inverse_kinematics` 不动）：枚举 shoulder×elbow×wrist 并 FK 校验每个候选；仍无解时用**自带的 DH 雅可比数值抛光**（同一 DH 模型，无 MuJoCo 依赖，真机可用）。**求解率 79% → 89%（枚举）→ 97%（+数值）**（500 随机可达构型）。
    - **⚠️ 整合进仿真执行 = 卡在 DH↔MuJoCo ~10mm 几何差**：把 MuJoCo 抓取目标经 `T_off`（腕部帧偏移，旋转部分完全恒定、平移含 ~10mm slop）转到 DH 帧后，目标落在解析可达范围外（0 分支），数值也难收敛 → 仿真里用解析 IK 驱动抓取**失败**。这正是 D7 一开始不用解析 IK 的原因。
    - **✅ 根因定位 + 解法（runhanw 提出，已验证）**：那 ~10mm **不是 DH 的问题，是仿真用了 Menagerie `agilex_piper`**（当初 D1 因 repo URDF 缺 STL mesh 才用的替代品）。实测 **runhanw 的 DH 和 repo 的 `config/piper_description.urdf` 运动学完全一致**——DH-FK 与 URDF-FK 差一个**恒定**变换（平移 std 0.03–0.07mm、旋转 std 1e-5），因为 DH 的 `a/d/theta_offset` 就是从这个 URDF 的 joint origin 取的（`d0=0.123`、`a2=0.28503`、`a3=-0.02198/d3=0.25075`、`d5=0.091`、`θoff2=-1.7939` 全对上）。**所以只要把仿真模型从 Menagerie 换成这个 URDF，DH-FK≈MuJoCo-FK（<0.1mm），解析 IK 就能精确驱动仿真 → 真·一套 IK。**
    - **✅ mesh 已搞定（2026-06-30）**：从 AgileX 官方开源仓库 [`agilexrobotics/piper_ros`](https://github.com/agilexrobotics/piper_ros)（noetic 分支 `src/piper_description/meshes`）下载标准版 STL（base_link/link1-8/gripper_base，10 个），落到 `sim/models/piper_real/meshes/`。用**我们的 URDF + 这些 mesh** 生成 `sim/models/piper_real/piper_real.urdf`（改 mesh 路径 + 加 `<mujoco><compiler meshdir>`），MuJoCo 加载 OK（nbody=9 njnt=8 nmesh=10，mesh 单位米）。**实测 DH-FK 与这个 MuJoCo 模型 link6 FK 差 <0.1mm（trans std 0.034–0.068mm）** → 模型和 DH 对上了，那 ~10mm 消失。
    - **剩余工作（换模型的代价）**：要真正让仿真跑在这个模型上，得**在新模型上重搭场景**：加 actuator（6 臂关节 + 夹爪）、夹爪 equality 约束、home keyframe、TCP site、并入桌子/YCB 物体/相机；且下游 tuning 要重来（`wrist_cam` 挂载位姿、观察/扫描位姿、抓取参数都是按 Menagerie 几何调的，link6 帧不同要重调）。相当于在新机器人模型上重做 Phase 0-2 + 抓取调参。
    - 现状：仿真管线未改动（只在 piper_arm.py 加了 solve_ik），pick 回归仍 113mm。IK 统一的**验证已通过、路已通**，剩下是新模型场景重搭这个体力活。
- **D8**（队友同步）：Phase 6 真·视觉抓取用 **`world_cam` 眼看手**（外参 `base_T_camera` 已有，最快跑通 Route B）；**`wrist_cam` 眼在手移到 Phase 7**（最贴近真机手眼标定，但需先解观察位姿）。task library 延后，实体机械臂待定。
### 8. 下一步
**最小闭环已完成（M1-M4）**。剩余为扩展项，可按需推进：

**Phase 6 — 完整视觉链路 & 接口（→ Milestone 5）**：
1. Route B：读 MuJoCo depth buffer，按 YOLO bbox 做 ROI 深度反投影 → camera frame 目标中心（复现真机 RealSense pipeline）。注意 camera(+y up,-z)→ROS optical(+y down,+z) 轴向转换（见 6.5）。
2. 用 Route B 的点替换 Route A，跑通完整 RGB-D 闭环。
3. 封装可被外部（未来 OpenHarmony command bridge）调用的 grasp action 入口（command format + I/O 日志）。

**可选打磨**：bottle 真实 mesh/贴图（让 YOLO 检出，D6）；眼在手 `wrist_cam` 求观察位姿（D5）；抓取鲁棒性（多物体位置、成功率统计）。

> 环境就绪：`MUJOCO_GL=egl`。相机=`world_cam`（D5），目标=cup（D6），IK=MuJoCo 雅可比 position-only（D7）。一键复现抓取：`MUJOCO_GL=egl python3 sim/grasp_sim.py`。
