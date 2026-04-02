# FYT Workspace

本仓库是独立云台激光跟随的最小工作区，保留 4 个核心包：
- `solver`：解算与跟随主节点（tracker / yolo / hybrid）
- `camera_driver`：大恒相机驱动
- `interfaces`：消息定义
- `utils`：基础依赖工具

## 快速开始

### 1) 编译

```bash
cd /home/zst/FYT
colcon build --packages-select interfaces utils camera_driver solver --parallel-workers 2
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
```

### 2) 运行

```bash
ros2 launch solver run.launch.py
```

独立云台模板参数：

```bash
ros2 launch solver gimbal.launch.py
```

## 文档分工

- 仓库总览和最短启动路径：当前文件
- 完整操作说明书（含 YOLO 接入、参数说明、调参步骤、故障排查）：`solver/README.md`
- 夜间参数模板与快速联调：`solver/README.md` 第 8 章

请优先阅读：
- `solver/README.md`

## 关键配置入口

- `solver/config/solver_gimbal_params.yaml`：解算与跟随主参数（模式、预测、门控）
- `solver/config/camera_driver_params.yaml`：相机帧率、曝光、增益

## 备注

- 若存在多个 ROS 工作区，请确认当前终端最终 source 到 `/home/zst/FYT/install/local_setup.bash`。
- 若你之前编译时叠加过其它工作区，优先使用 `source /home/zst/FYT/install/local_setup.bash`，可避免旧 underlay 干扰。
# little
