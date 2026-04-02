# solver

`solver` 是独立云台场景的 C++ ROS2 解算节点，支持三种模式：
- `tracker`：手动框选后高频跟踪
- `yolo`：每帧 YOLO 检测
- `hybrid`：低频 YOLO 重定位 + 高频 Tracker（推荐）

节点输出：
- `armor_solver/cmd_gimbal` (`interfaces/msg/GimbalCmd`)
- `armor_solver/laser_debug` (`interfaces/msg/DebugLaser`)

---

## 1. 这份说明书怎么用

如果你想最快跑起来，按下面顺序看：
1. 先看“2. 快速启动”
2. 然后看“3. YOLO 接入说明”
3. 实机抖动就看“6. 调参顺序”
4. 出问题直接看“7. 常见问题”

---

## 2. 快速启动

### 2.1 编译

```bash
cd /home/zst/FYT
colcon build --packages-select solver --parallel-workers 2
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
```

### 2.2 启动

推荐（相机 + solver 一键启动）：

```bash
ros2 launch solver run.launch.py
```

独立云台参数模板（推荐实机联调）：

```bash
ros2 launch solver gimbal.launch.py
```

仅启动 solver（相机需已运行）：

```bash
ros2 launch solver solver.launch.py
```

对应参数文件：
- `solver/config/solver_gimbal_params.yaml`

---

## 3. YOLO 接入说明（重点）

### 3.1 模型要求

- 模型格式：ONNX
- 推荐输入尺寸：`640x640`
- 输出格式：YOLO 常见检测头（本节点已做解码 + NMS）
- 类别：可多类别，也可单类别

### 3.2 参数配置

编辑 `solver/config/solver_gimbal_params.yaml`：

1. 设置模式
- `detector.mode: "hybrid"`（推荐）
- 或 `detector.mode: "yolo"`（算力足够时）

2. 设置模型路径
- `detector.yolo.model_path: "/home/zst/FYT/model/yolo.onnx"`

3. 设置阈值
- `detector.yolo.conf_thres: 0.35`
- `detector.yolo.nms_thres: 0.45`

4. 类别筛选（可选）
- `detector.yolo.target_class_id: -1` 表示不限制类别
- 如果只打某一类，改成对应类别 id

单实例双颜色项目建议（同一目标两类，如 red_target / blue_target）：
- `detector.yolo.single_target_mode: false`
- `detector.yolo.target_class_id: -1`
- `detector.yolo.class_color_gate.enabled: true`
- `detector.yolo.class_color_gate.red_class_id: <你的红色类ID>`
- `detector.yolo.class_color_gate.blue_class_id: <你的蓝色类ID>`

说明：
- 该配置会允许两种类别进入候选，但每帧最终只输出一个目标（由选择器打分决定）。
- 节点会根据 `color_filter.self_color` 自动推导敌方颜色，只放行敌方颜色类别。
- 若你改成单类别训练（只保留 target 一类），再设置 `single_target_mode: true` 与 `target_class_id: 0`。

敌我声明方式：
- 开机时在参数里声明自己是红方或蓝方：`color_filter.self_color: red/blue`
- 运行中也可用快捷键临时切换：`b/e/a`（见下文）

5. hybrid 关键参数
- `detector.yolo.detect_interval: 3`
	- 更实时：调大（4~5）
	- 更稳不易漂：调小（2）

### 3.3 启动验证

```bash
source /opt/ros/jazzy/setup.bash
source /home/zst/FYT/install/local_setup.bash
ros2 launch solver gimbal.launch.py
```

验证点：
1. 图像窗口有检测框
2. `armor_solver/cmd_gimbal` 持续发布
3. 目标短时遮挡时不立刻丢（predict hold 生效）

可选检查命令：

```bash
ros2 topic hz /armor_solver/cmd_gimbal
ros2 topic echo /armor_solver/laser_debug --once
```

### 3.4 什么时候用哪种模式

- `tracker`：手动选目标，追踪最轻量
- `yolo`：全自动，算力压力最大
- `hybrid`：实战推荐，实时性和鲁棒性平衡最好

### 3.5 重做数据集入口

已预留训练说明目录：
- `solver/training/README.md`

你可以在这里按单目标（class 0）重做数据集，并导出 ONNX 后接回本项目。

---

## 4. 使用流程

### 4.1 tracker 模式

1. 启动后会出现 `simple_tracker` 窗口。
2. 鼠标左键框选目标。
3. 按 `Enter` 或 `Space` 确认。
4. 节点开始持续跟随并发布云台控制。
5. 按 `r` 重新框选。

快捷键：
- `b`：敌方颜色强制蓝色
- `e`：敌方颜色强制红色
- `a`：敌方颜色自动

### 4.2 yolo / hybrid 模式

- `yolo`：自动检测，无需手动框选。
- `hybrid`：YOLO 每 N 帧校正一次，其余帧走 tracker，实时性更好。

---

## 5. 当前核心能力

- 目标中心瞄准（框中心）
- 颜色门控（敌我红蓝）
- 距离门控（10-25m）
- 激光-相机刚性安装补偿
- 轨迹预测（前馈预瞄）
- 线运动抗抖（单轴约束）
- 丢检短时保持（predict hold）
- 计分稳定窗口（命中稳定后再累计）

---

## 6. 调参顺序（实机推荐）

参数文件：`solver/config/solver_gimbal_params.yaml`

1. 先调方向
- `yaw_sign` / `pitch_sign`

2. 再调增益
- `yaw_gain` / `pitch_gain`

3. 再调预测
- `prediction.lead_time_s`
- `prediction.max_lead_px`

4. 再调抗抖
- `line_motion.axis`
- `prediction.vel_jump_thres_px_s`

5. 最后调命中判定
- `lost_hold.hold_s`
- `score.stability_window_s`

### 6.1 检测相关

- `detector.mode`: `hybrid` / `yolo` / `tracker`
- `detector.yolo.model_path`: ONNX 路径
- `detector.yolo.detect_interval`: hybrid 下 YOLO 间隔帧数（推荐 2~4）
- `detector.yolo.single_target_mode`: 单目标模式开关
- `detector.selector.*`: YOLO多候选目标选择器（精准自动跟随核心）

推荐先调这 3 个：
1. `detector.selector.w_center`
2. `detector.selector.w_iou`
3. `detector.selector.min_area_px`

调参建议：
- 目标跳来跳去：增大 `w_iou`（如 0.20 -> 0.30）
- 总想吸到画面中心附近干扰目标：减小 `w_center`
- 远处小目标漏跟：减小 `min_area_px`

### 6.2 预测与抗抖

- `prediction.lead_time_s`: 预瞄时间（过冲大就减小）
- `prediction.max_lead_px`: 预瞄上限像素（抖动大就减小）
- `prediction.vel_jump_thres_px_s`: 速度突变阈值
- `line_motion.axis`: `x` 或 `y`（目标主要运动方向）

### 6.3 丢检保持与计分稳定

- `lost_hold.hold_s`: 丢检保持时长（推荐 0.15~0.25）
- `score.stability_window_s`: 命中稳定窗口（推荐 0.08~0.15）

### 6.4 云台方向与增益

- `yaw_sign` / `pitch_sign`: 方向不对先改符号
- `yaw_gain` / `pitch_gain`: 再调增益

---

## 7. 常见问题

### 7.1 不实时/卡顿

优先检查 `solver/config/camera_driver_params.yaml`：
- 降低 `exposure_time`
- 合理设置 `frame_rate`

并确认：
- 没有其他程序占用相机
- 终端环境已正确 source：

```bash
source /opt/ros/jazzy/setup.bash
source /home/zst/FYT/install/local_setup.bash
```

### 7.2 跟随方向反了

先改：
- `yaw_sign`
- `pitch_sign`

### 7.3 高频抖动或过冲

按顺序调小：
1. `prediction.max_lead_px`
2. `prediction.lead_time_s`
3. `yaw_gain` / `pitch_gain`

### 7.4 YOLO 不出框

按顺序排查：
1. 模型路径是否正确：`detector.yolo.model_path`
2. 模型是否真的是检测 ONNX（不是分类模型）
3. 降低阈值：`conf_thres` 先试 `0.2`
4. 类别是否设错：`target_class_id` 先改 `-1`
5. 先切 `detector.mode: yolo` 单独验证，再回 `hybrid`

### 7.5 串口如何接回 FYT 原版

当前仓库没有 `serial_driver` 源码包（你说的原版在压缩包里）。

为了提前对接，`solver` 已预留串口桥接输出：
- 参数：`serial_bridge.enabled`
- 话题：`serial/tx_packet`（`std_msgs/msg/UInt8MultiArray`）

字节帧格式（19字节）：
1. `0xA5 0x5A`
2. `ver=0x01`
3. `flags`：bit0=laser_power_on, bit1=laser_hit_valid
4. `yaw(float32 LE)`
5. `pitch(float32 LE)`
6. `distance(float32 LE)`
7. `checksum`：前16字节异或
8. `0x0D 0x0A`

你把 FYT 原版串口节点恢复后，只要订阅 `serial/tx_packet` 并按原串口协议转发即可。

---

## 8. 夜间参数模板（可直接抄）

你在夜间场景（你当前也在用 `camera_profile:=night`）可以先用这套起步参数，再按 6 章顺序微调。

### 8.1 相机参数模板

文件：`solver/config/camera_driver_params.yaml`

```yaml
/**:
	ros__parameters:
		frame_rate: 180
		exposure_time: 1800
		gain: 18.0
```

说明：
- 夜间先把 `exposure_time` 控在 1500~2200，避免拖影。
- 画面太暗再加 `gain`，不要优先拉高曝光。
- 若电机高速抖动明显，先把 `frame_rate` 保在 150~180 稳定段。

### 8.2 solver 参数模板（night 起步）

文件：`solver/config/solver_gimbal_params.yaml`

```yaml
/**:
	ros__parameters:
		detector:
			mode: "hybrid"
			yolo:
				conf_thres: 0.28
				nms_thres: 0.45
				detect_interval: 3

		prediction:
			lead_time_s: 0.07
			max_lead_px: 70.0
			vel_jump_thres_px_s: 220.0

		lost_hold:
			hold_s: 0.22

		score:
			stability_window_s: 0.10

		yaw_gain: 0.75
		pitch_gain: 0.75
```

说明：
- 夜间 YOLO 置信度可适当下调（如 0.28~0.32）提升检出率。
- 若出现过冲，优先降 `max_lead_px`，再降 `lead_time_s`。
- 若频繁掉目标，先把 `hold_s` 提到 0.25，再考虑降 `detect_interval`。

### 8.3 5 分钟快速联调

1. 启动：`ros2 launch solver gimbal.launch.py`
2. 看框是否稳定：框抖就先调相机曝光和增益。
3. 看跟随是否超前过冲：过冲就降 `max_lead_px`。
4. 看遮挡后是否能续上：续不上就加 `hold_s`。
5. 看计分是否过严或过松：调 `stability_window_s`。
