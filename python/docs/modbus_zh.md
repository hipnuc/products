# Modbus RTU

[English](modbus.md) | [中文](modbus_zh.md)

默认 `115200 / 8N1`，站号 `80`，单播站号 `1–247`，仅使用功能码 FC03（读）和
FC06（写）。一条物理总线只创建一个 `ModbusBus`，总线上的所有站号共用串口和事务锁。
写请求不会自动重发，也不会自动追加保存请求。

## 单机准备

1. 先只接一台支持 RS-485/Modbus 的设备，核对供电、A/B 接线、当前波特率和站号。
2. 按型号手册确认哪个 COM 口提供 Modbus RTU。
3. 若该口正在主动输出串口帧，用单机 ASCII 会话关闭输出：
   `hipnuc command "LOG DISABLE" -p COM3 -b 115200`（要重启后仍关闭，请关掉定时消息并
   发送 `SAVECONFIG`）。
4. 关闭 ASCII 会话，再运行 `hipnuc modbus info -p COM3 --id 80`。
5. 多机合并总线前，逐台设置唯一站号和一致波特率。多机总线上不要运行 `scan` 或 `command`。

## 读取

```sh
hipnuc modbus info -p COM3 --id 80
hipnuc modbus read -p COM3 --id 80
hipnuc modbus read -p COM3 --id 80 --duration 60 --record samples.jsonl
hipnuc modbus read -p COM3 --id 80 --count 10 --interval 0.1 --jsonl
```

`-b` 默认 115200，`--id` 默认 80，`--timeout` 默认 2 秒。`read` 连续轮询直到 Ctrl-C、
`--duration` 或 `--count`；`--interval` 是两次轮询之间的等待时间。

```python
from hipnuc import ModbusBus

with ModbusBus("/dev/ttyUSB0") as bus:
    device = bus.device(80)
    info = device.read_info()
    status = device.read_status()  # main_status、status_flags、校准状态
    sample = device.read_sample(include_status=True, include_mru=False)
    words = device.read_registers(0x34, 26)
```

| 数据块 | 寄存器 |
| --- | --- |
| 身份 | `0x70–0x82`（19 个寄存器） |
| 主状态、校准状态及进度 | `0x09–0x0B` |
| IMU/姿态 | `0x34–0x4D`（26 个寄存器） |
| 含 MRU 升沉/纵荡/横荡 | `0x34–0x53`（32 个寄存器） |

寄存器为大端、零起始地址，32 位量高字在前。加速度每计数 `9.8 / 2048 m/s²`，磁场
32.768 counts/µT。`sample.metadata["register_snapshot"]` 保持 `not_guaranteed`：
同一数据块内的字段不保证来自同一固件周期。

## 读写寄存器

```sh
hipnuc modbus registers 0x06 1 -p COM3 --id 80
hipnuc modbus write-register 0x06 1 -p COM3 --id 80 --save --reboot
```

直接使用产品手册中的寄存器地址和值。`write_register` 默认读回（只写项用
`--no-verify` / `verify=False`）。非法写入设备也可能正常回显，只有读回匹配才
`verified=True`。`--save` 在写入成功后保存一次，`--reboot` 再重启并等待设备恢复。

## 保存、改站号、改波特率

```python
with ModbusBus("COM3") as bus:
    device = bus.device(80)
    device.write_register(0x06, 1)  # 航向模式，见手册
    device.write_register(0xA6, 24)  # 安装方向
    device.save_config()  # 批量配置后保存一次
    device.reboot()
```

- `set_id(81, save=False)`：写入新站号，对象改用新地址并在新地址读回站号寄存器。
  目标站号必须在总线上空闲。
- `set_baudrate(921600, reboot=False, save=False)`：写入波特率代码，设备重启后生效。
  `reboot=True` 时 SDK 会重启、把主机串口切到新速率并等待身份块可读。
- `reboot(timeout=5, save=False, baudrate=None)`：发送一次复位并等待 `read_info()` 应答。
  新波特率生效时传入 `baudrate`。
- `bus.reconfigure(baudrate)`：只修改主机速率。

```sh
hipnuc modbus set-id 81 -p COM3 --id 80 --save
hipnuc modbus baudrate 921600 -p COM3 --id 81 --save --reboot
hipnuc modbus info -p COM3 -b 921600 --id 81
```

同一总线上所有站共用波特率，修改某一站不会替其他站配置。

## 在程序中录制

```python
from hipnuc import ModbusBus, Recorder

with Recorder("samples.jsonl") as recording, ModbusBus("COM3") as bus:
    device = bus.device(80)
    for _ in range(100):
        recording.write(device.read_sample())
```

每条记录保留 `metadata.device_id`。多站轮询见
[modbus_multinode.py](../examples/modbus_multinode.py)：修改文件顶部的 `PORT`、
`BAUDRATE`、`NODE_IDS`、`INTERVAL_S`，在 `python/` 目录运行。

寄存器地址和取值以产品的指令与编程手册为准。
