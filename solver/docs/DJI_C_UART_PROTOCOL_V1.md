# DJI C UART Protocol V1

本协议用于上位机与 DJI C板之间的双向 UART 通信。

## 1. 传输建议

- 物理层：UART TTL
- 波特率：115200 或 460800（建议 115200 先联调）
- 数据格式：8N1
- 发送频率：100 Hz ~ 300 Hz

## 2. 帧格式（24 bytes）

Byte0   : SOF = 0xA5
Byte1   : Version = 0x01
Byte2   : PayloadLen = 0x12
Byte3   : Seq (uint8, 自增)
Byte4-7 : yaw (float32, little-endian, deg)
Byte8-11: pitch (float32, little-endian, deg)
Byte12-15: yaw_rate (float32, little-endian, deg/s)
Byte16-19: pitch_rate (float32, little-endian, deg/s)
Byte20  : flags (uint8)
          bit0 laser_power_on
          bit1 laser_hit_valid
          bit2 lock_active
          bit3 fire_advice
Byte21  : reserved (uint8, 当前固定 0)
Byte22  : CRC8 (poly 0x07, init 0x00, 计算范围 Byte0..Byte21)
Byte23  : EOF = 0x5A

## 3. 坐标约定

- yaw: 左正右负（deg）
- pitch: 下正上负（deg）
- 与 FYT 当前控制链保持一致。

## 4. C板解析伪代码

```c
bool parse_frame(const uint8_t *buf, int n, Cmd *out) {
  if (n < 24) return false;
  if (buf[0] != 0xA5 || buf[23] != 0x5A) return false;
  if (buf[1] != 0x01 || buf[2] != 0x12) return false;

  uint8_t crc = crc8_poly07(buf, 22); // Byte0..Byte21
  if (crc != buf[22]) return false;

  out->seq = buf[3];
  out->yaw = read_f32_le(buf + 4);
  out->pitch = read_f32_le(buf + 8);
  out->yaw_rate = read_f32_le(buf + 12);
  out->pitch_rate = read_f32_le(buf + 16);

  uint8_t flags = buf[20];
  out->laser_on = (flags & 0x01) != 0;
  out->laser_hit_valid = (flags & 0x02) != 0;
  out->lock_active = (flags & 0x04) != 0;
  out->fire_advice = (flags & 0x08) != 0;
  return true;
}
```

## 5. 上位机兼容旧协议

- `serial_bridge.protocol = dji_c_v1`：使用本协议
- `serial_bridge.protocol = legacy19`：保留历史 19 字节协议

## 6. 回传帧（C板 -> 上位机）

回传帧同样使用 24 bytes 定长结构，便于统一解析。

Byte0   : SOF = 0xA5
Byte1   : Version = 0x01
Byte2   : PayloadLen = 0x12
Byte3   : Seq (uint8, C板自增)
Byte4-7 : yaw (float32, little-endian, deg)
Byte8-11: pitch (float32, little-endian, deg)
Byte12-15: yaw_rate (float32, little-endian, deg/s)
Byte16-19: pitch_rate (float32, little-endian, deg/s)
Byte20  : flags (uint8)
          bit7 enemy_air_support_active
Byte21  : mode (uint8)
          0: red, 1: blue（与 `SerialReceiveData.mode` 对齐）
Byte22  : CRC8 (poly 0x07, init 0x00, 计算范围 Byte0..Byte21)
Byte23  : EOF = 0x5A

ROS 映射：

- 发布话题：`serial/receive`
- 消息类型：`interfaces/msg/SerialReceiveData`
- 字段映射：
  - `mode <- Byte21`
  - `enemy_air_support_active <- bit7(Byte20)`
  - `yaw <- Byte4-7`
  - `pitch <- Byte8-11`
  - `roll <- 0`
  - `bullet_speed <- 0`（后续可扩展）

## 7. 联调顺序建议

1. C板先只打印 yaw/pitch，不闭环
2. 确认角度方向正确后再闭环
3. 再启用 yaw_rate/pitch_rate 作为前馈
4. 最后启用 flags 相关逻辑
