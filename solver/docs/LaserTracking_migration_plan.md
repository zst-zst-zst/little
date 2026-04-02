# LaserTracking-2026 迁移到 FYT（DJI C板）

## 1. 已确认可复用模块（可直接迁）

### 1.1 检测后端
- 来源：`LaserTracking-2026-main/src/detector`（TensorRT + TRTInferX）
- FYT 现状：已接入 TensorRT 引擎路径与推理分支。
- 注意：T 解码器默认期望输出布局为 `[1, 8400, C]`；FYT 训练导出常见为 `[1, C, 8400]`，需在导出后做输出转置。

### 1.2 控制器逻辑（追踪核心）
- 来源：`LaserTracking-2026-main/src/control/include/control/controller.h`
- 来源：`LaserTracking-2026-main/src/control/src/controller.cpp`
- 核心能力：
  - 像素误差到角度误差映射（atan + 内参）
  - P 控制 + 限速 + 低通
  - 速度前馈（meas uv）
  - 阻尼项（meas 或 gimbal 角速度）
  - 丢失目标搜索（circle/spiral）
  - 启动阶段状态机（hold/home/validate）

### 1.3 串口读写基础封装
- 来源：`LaserTracking-2026-main/src/gimbal_serial/src/serial_port.cpp`
- 能力：termios、超时读、重开串口、状态统计。
- 该层可迁移，但应避免直接绑定其上层协议。

## 2. 不可直接复用模块（需改造）

### 2.1 上位机-下位机协议帧
- 来源：`LaserTracking-2026-main/src/gimbal_serial/src/protocol.cpp`
- 原协议：22字节，帧头 0xCD，帧尾 0xDC，中间直接 float + uint32（无 CRC 字段）
- 不可直接用于 DJI C板：
  - C板协议通常有自身帧结构、字段定义、校验方式、模式位
  - 需要按 C板协议重新打包 `pitch/yaw/rate/flags` 并映射单位

### 2.2 相机模块
- T 使用海康链路（`hik_camera`），FYT 为大恒相机链路
- 不迁移相机模块，仅迁移检测/控制/通信上层逻辑

## 3. FYT 目标架构（建议）

1) `detector`：TensorRT 输出目标框 + 类别 + 置信度
2) `tracker/controller`：使用 T 风格 Controller 计算云台指令
3) `serial_adapter_dji_c`：将 `GimbalCmd` 转成 DJI C板协议帧
4) `serial_driver`：按既有 FYT 串口节点收发

## 4. 当前已完成状态（FYT）

- 已接入 TensorRT 推理路径
- 已支持 T 风格关键筛选：
  - `allowed_class_ids`
  - `max_targets_for_valid`
- 已修复 ONNX 输出布局与 T 解码器不一致问题（通过转置模型输出）

## 5. 下一步落地顺序（按优先级）

### P0：控制器迁移（优先）
- 从 T 的 `Controller` 迁移最小闭环：
  - P + 限速 + 低通 + deadband
  - 基础 lost-scan（circle）
- 接入 FYT 参数文件并保持可热调

### P1：前馈与阻尼
- 加入 `use_velocity_ff` 和 `use_damping` 两项
- 先使用 meas 源，后可切 gimbal 反馈源

### P2：启动状态机
- 迁移 startup hold/home/validate 流程
- 上电稳定后再允许搜索和开火建议

### P3：DJI C板协议适配
- 新增 `serial_adapter_dji_c`
- 输入：`interfaces/msg/GimbalCmd`
- 输出：`serial/tx_packet`（按 C板协议打包）

## 6. 协议对接所需信息（DJI C板）

必须提供以下字段定义后，方可一次性完成协议适配：
- 帧头/帧尾、长度、端序
- 角度/角速度单位（deg/rad、缩放倍率）
- 模式位定义（激光、锁定、开火建议）
- 校验方式（CRC8/CRC16/XOR）
- 回传帧字段（当前 yaw/pitch、模式、时间戳）

## 7. 验证标准（验收）

- 静态目标：0.5s 内稳定收敛，无抖动发散
- 低速横移：连续跟踪不中断
- 丢失重获：进入搜索后可在 2s 内重获
- 串口链路：指令和回传时间戳单调、无错帧暴涨
