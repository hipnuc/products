[English](README.md) | [中文](README_zh.md)

# CAN 数据库

[J1939.dbc](J1939.dbc) 描述 HiPNUC 的 J1939 数据报文，供 CANalyzer、CANoe、
PCAN-Explorer、asammdf、MATLAB 等 CAN 分析工具使用。不用数据库也可以直接看数值：
运行 `hihost can read`（见 [python/](../python/README_zh.md)）。

## 包含的内容

- 13 个数据 PGN，0xFF10 到 0xFF4A：Classic CAN、29 位标识符、8 字节、小端。
- IMU 模块默认每 10 ms 发送加速度（FF34）、角速度（FF37）、横滚/俯仰（FF3D）和航向
  （FF41）。其他 PGN 用 `hihost can period` 打开，例如打开四元数（FF46）：
  `hihost can period quat 10 -i can0 --id 8`。
- FF10、FF14、FF18、FF26 来自组合导航型号，FF3A 来自带磁力计的型号，FF43 不是每个
  型号都发送。
- 不包含 CAN FD 型号的 CANFD83 帧，请用 Python 或 C 解码器解码。

## 节点地址

标识符使用优先级 3 和出厂节点地址 8，例如 `0x0CFF3408`。数据库已标记为 J1939，
支持 J1939 数据库的工具按 PGN 匹配，任何节点地址的模块都能解码。使用 cantools 时，
屏蔽优先级和地址：

```sh
candump can0 | python3 -m cantools decode -m 0x03FFFF00 J1939.dbc
```

```python
import cantools

db = cantools.database.load_file("J1939.dbc", frame_id_mask=0x03FFFF00)
values = db.get_message_by_frame_id(msg.arbitration_id).decode(msg.data)
```

按完整标识符比较的工具只能解码节点地址 8；其他地址请在工具中修改标识符。

## 数值

- 加速度单位为 G（固件按 1 G = 9.8 m/s²），角速度为 deg/s，磁场为 µT，角度为度。
- `Heading_CW` 为顺时针 0 到 360°；`Euler_Yaw` 为输出轴向下的 ±180°。`COORD 0`
  时两者表示同一航向。
- FF2F：模块获得 UTC 之前，年、月、日为 0，时间字段为开机时间，按 24 小时循环。
- FF18 的质量和状态字段带取值名称。

PGN 列表、发送周期和配置协议见对应产品的 IMU 或 INS 手册。
