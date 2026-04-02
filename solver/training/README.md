# 单目标数据集与训练预留位

本目录用于你后续重做单目标数据集与训练模型。

## 1) 目录约定

建议结构（YOLO 格式）：

```text
solver/training/
  datasets/
    target_v1/
      images/
        train/
        val/
      labels/
        train/
        val/
      data.yaml
  exports/
    onnx/
      target_v1.onnx
```

## 2) 类别约定（单目标）

- `class 0`：你的唯一目标
- 在 `solver/config/solver_gimbal_params.yaml` 保持：
  - `detector.yolo.single_target_mode: true`
  - `detector.yolo.target_class_id: 0`

## 3) 部署位置

训练后将 ONNX 放到：

- `/home/zst/FYT/model/yolo.onnx`（当前默认路径）

或者改参数：

- `detector.yolo.model_path`

## 4) 最小检查清单

1. 模型输入尺寸与参数 `input_w/input_h` 一致。
2. 模型输出为检测头（不是分类模型）。
3. 若漏检：先降 `conf_thres` 到 0.2~0.3。
4. 若误检多：提高 `conf_thres` 或增大 `w_iou`。
