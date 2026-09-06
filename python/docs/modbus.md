# Modbus RTU

默认 `115200 / 8N1`，站号 `80`，单播范围 `1–247`。一个物理总线只创建一个
`ModbusBus`，所有站号共用其串口与事务锁，包含配置后读回的完整操作。
写请求不会自动重复发送，也不会自动追加保存请求。配置是否写入后立即持久化取决于固件；
为兼容需要显式保存的旧固件，批量配置完成后调用一次 `save_config()`。

以下 CLI 示例使用 `hipnuc` 简写。Windows PowerShell 按快速开始安装后，将其替换为
`.\.venv\Scripts\python.exe -m hipnuc`；使用安装时的同一个 Python，
不要求 Scripts 目录在 PATH 中。

For the CLI examples below, Windows PowerShell users can replace `hipnuc` with
`.\.venv\Scripts\python.exe -m hipnuc`. With the Python used for installation,
`python -m hipnuc` also works without the Scripts directory on PATH.

**Quick start:** connect one RTU-capable device, use its configured baudrate and
node ID, and run `hipnuc modbus info -p COM3 --id 80`, then replace `info` with `read`.
For several nodes, create one `ModbusBus` and call `bus.device(id)` for each unique
address. Disable unsolicited serial output before joining the bus; send ASCII
configuration commands only with a single device connected. The SDK does not
automatically send a save request. Persistence depends on the firmware; call
`save_config()` once after a batch for compatibility with older firmware. The Python
examples below work on Windows, Linux and macOS with the appropriate port path.

## 单机准备

1. 先只接一台支持 RS-485/Modbus 的设备，核对供电、接线、当前波特率和站号。
2. 按型号手册确认 RTU 所在端口；不能假设任意产品的每个 COM 都提供 Modbus。
3. 若该端口正在主动输出串口帧，用单机 ASCII 会话发送 `LOG DISABLE` 停止当前
   输出；通过 `LOG COMCONFIG` 查看消息配置。若要重启后仍不主动输出，按该型号
   的公开 LOG 语法关闭已配置的定时消息，最后执行一次 `SAVECONFIG` 并读回。
   `LOG DISABLE` 是临时输出开关；持久设置以定时消息配置为准。
4. 关闭 ASCII 会话，再用 `hipnuc modbus info -p COM3 --id 80` 查询身份。
5. 多机合并总线前，逐台配置唯一站号和一致波特率。连接多机后仅发有地址 RTU，
   不运行 ASCII `scan`、`command` 或自动广播改号。

`ModbusBus(port, baudrate=115200, timeout=0.5, handle_local_echo=False)` 可用于
`with`，或显式 `open()/close()`。转接器确实将 TX 回显到 RX 时才启用
`handle_local_echo=True`；这与设备返回 FC06 回显是两回事。

## 读取

CLI 的连接选项统一放在最终子命令之后。Modbus 必须指定端口，不自动扫描站号；
`-b/--baudrate` 默认为 115200，`--id` 默认为 80，CLI 的 `--timeout` 默认为 2 秒：

```sh
hipnuc modbus info -p COM3 --id 80
hipnuc modbus read -p COM3 --id 80
hipnuc modbus read -p COM3 --id 80 --duration 60 --record samples.jsonl
hipnuc modbus read -p COM3 --id 80 --count 10 --interval 0.1 --jsonl
```

`read` 默认连续轮询，Ctrl-C 停止。`--duration` 或 `--count` 可限制采集长度，
同时指定时先满足者生效；`--interval` 是两次轮询之间的等待时间。
停止条件按已完成样本判断：到达时限后不再发起新轮询，已经开始的事务会完成，
因此最后一次轮询可能在指定时长之后结束。
屏幕默认每种报文每秒最多显示 5 次，JSONL 文件记录全部返回样本并保留
`metadata.device_id` 站号。

```python
from hipnuc import ModbusBus

with ModbusBus("/dev/ttyUSB0") as bus:
    device = bus.device(80)
    info = device.read_info()
    status = device.read_status()
    sample = device.read_sample(include_status=True, include_mru=False)
    words = device.read_registers(0x34, 26)
```

| 操作 | 地址 / 范围 |
| --- | --- |
| 身份 | `0x70–0x82`，一次 19 寄存器 |
| 主状态、校准状态及进度 | `0x09–0x0B` |
| IMU/姿态 | `0x34–0x4D`，一次 26 寄存器 |
| 连同 MRU | `0x34–0x53`，一次 32 寄存器 |

FC03 每次 `1–125` 个寄存器，地址为零起始地址。宽字段放在同一请求，按大端字节、
高字在前解码。高层首次使用会查询身份，未知型号/版本继续使用共同公开协议。
加速度按产品协议的 `1 G = 9.8 m/s²` 换算，每个计数为 `9.8 / 2048 m/s²`，
与 HI91、HI83 的 SI 输出保持一致，整型量化误差除外。
INS 可以读取支持的共同块；位置、速度等未公开为 Modbus 寄存器的 INS 功能不作推断，
使用串口 HI81/HI83/NMEA 获取。

`timeout` 是设备处理和主机等待余量，每次事务再按当前波特率加上请求、响应及帧间隔
所需时间；短请求不会沿用上一个长请求的预算。一次读取连续块减少事务开销，但**不保证旧固件
内部字段来自同一采样时刻**，`metadata.register_snapshot="not_guaranteed"` 会保留。
状态块与测量块也是不同请求。未定义地址可能返回零，因此不能通过零值判断能力。

磁场默认 `documented`：32.768 counts/µT。历史固件可选
`bus.device(80, magnetic_scale="legacy")`：32 counts/µT。
两者都输出 Tesla，metadata 保留所用比例。该差异跨越同一 1.7.2 版本，不能仅靠
APP_VERSION 自动识别；请用设备记录确认后选择，其他字段不因此阻断。

## 寄存器读写

直接使用产品手册中的寄存器地址和值，无需另一套配置名称或命令语法。
SDK 负责 FC03/FC06 事务、可选读回，以及保存、重启和连接参数变化后的恢复。

```sh
hipnuc modbus registers 0x06 1 -p COM3 --id 80
hipnuc modbus write-register 0x06 1 -p COM3 --id 80 --save --reboot
```

此例将航向模式寄存器设为 `1`；实际支持的值、适用型号和生效条件以产品手册为准。
地址和值支持十进制及 `0x` 十六进制写法。

`write_register(address, value, verify=True)` 是原始 FC06，不保存、不重启、
不自动改变设备对象的站号；写只写控制项须显式 `verify=False`。
不将两个 FC06 拼成一个设备不支持的 32/64 位写入。

CLI `write-register` 默认读回；只写项使用 `--no-verify`。`--save` 在写入成功后保存一次，
`--reboot` 再重启并等待重新连接。任何步骤失败就停止，不执行后续步骤，之前的修改不会回滚。
原始控制寄存器 `0x00`、波特率 `0x04` 和站号 `0x05` 的写入不能附加
`--save` / `--reboot`；请使用下面的 `reboot`、`baudrate`、`set-id` 命令管理连接变化。

`WriteResult` 包含 `address/value/acknowledged/verified/readback`。旧固件非法写也可能
正常回显，只有读回匹配才 `verified=True`；ACK 丢失但可读字段已经生效时，
可返回 `acknowledged=False, verified=True`，不重发原写请求。
Modbus 异常、超时和读回不一致分别抛出相应异常。
保存与姿态复位是只写操作，应答不能证明闪存持久化或物理效果。

## 保存、改站号、改波特率

```python
with ModbusBus("COM3") as bus:
    device = bus.device(80)
    device.write_register(0x06, 1)  # Heading mode; see the product manual.
    device.write_register(0xA6, 24)  # Installation orientation.
    device.save_config()  # Save once after the batch.
    device.reboot()  # Reboot without another save.
```

- `set_id(81, save=False)`：收到旧地址应答后改用新地址，确认原设备身份及 ID 读回。
  目标地址必须事先空闲；SDK 不自动扫描或重编号。结果不明确时检查原/新地址。
- `set_baudrate(921600, reboot=False, save=False)`：默认只写配置并读回，主机保持原速率。
  `save=True, reboot=True` 明确执行保存、重启、主机换速、身份与寄存器验证。
- `reboot(timeout=5, save=False)`：发送一次复位请求；容忍旧固件复位前无 ACK，
  等待后重新读取身份。对象记录的待生效波特率随本次恢复使用。
- `bus.reconfigure(baudrate)`：只修改主机，可在已知设备参数下重新建立通信。

```sh
hipnuc modbus set-id 81 -p COM3 --id 80 --save
hipnuc modbus baudrate 921600 -p COM3 --id 81 --save --reboot
hipnuc modbus info -p COM3 -b 921600 --id 81
hipnuc modbus reboot -p COM3 -b 921600 --id 81 --save
```

多站总线共用速率，修改某一站并重启不会替其他站配置。先逐台准备或设计自己的总线
切换顺序；SDK 不执行广播改速。在最终命令后加 `-p`、`-b`、`--id`；通过 `--help` 查看参数。

## 在程序中录制

```python
from hipnuc import ModbusBus, Recorder

with Recorder("samples.jsonl") as recording, ModbusBus("COM3") as bus:
    device = bus.device(80)
    for _ in range(100):
        recording.write(device.read_sample())
```

`Recorder` 使用与串口相同的 JSONL 格式。Modbus 不提供原始 RTU 总线帧录制；
`sample.raw` 仅包含响应中的寄存器数据。CLI 不提供 `--record-raw`。
连续多站轮询见 [modbus_multinode.py](../examples/modbus_multinode.py)。
修改文件顶部的 `PORT`、`BAUDRATE`、`NODE_IDS`、`INTERVAL_S` 后，在 SDK 的 `python/`
目录运行 `python examples/modbus_multinode.py`，按 Ctrl-C 停止。
默认端口为 `COM3`、波特率为 115200，轮询站号 80、81，每轮等待 0.1 秒；例程不录制文件。

型号支持的命令和寄存器以官方 [IMU 手册](https://download.hipnuc.com/products/imu/cum.html)
和 [INS 手册](https://download.hipnuc.com/products/ins/cum.html) 为准。
