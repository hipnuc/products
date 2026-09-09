# HiPNUC Python SDK

[English](README.md) | [中文](README_zh.md)

在命令行或自己的 Python 程序中读取、配置和录制 HiPNUC IMU/AHRS/MRU 与 INS。
支持 HI91/HI81/HI83、NMEA GGA/RMC 和 Modbus RTU，适用于 **Python 3.10–3.14**、
Windows、Linux（含 Ubuntu 和树莓派系统）及 macOS。

可选 CAN 支持（J1939/CANFD83）基于 python-can。CAN 命令行使用 Linux SocketCAN；
Python 程序可以使用其他适配器。固件升级支持串口和 CAN。

支持设备：固件 1.7.0 及以上（HI01–HI06、HI12–HI18、HI32、HI70/HI71、CH0X0）。
不支持早期 1.7.1 中使用 4 字节时间戳的 HI83 布局。

## 安装

解压仓库，在其中的 `python/` 目录打开终端。

**Windows PowerShell：**

```powershell
py -3 --version
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install .
hihost --version
hihost --help
```

**Linux / 树莓派 / macOS：**

Ubuntu/Debian/Pi OS 缺少 venv 支持时，执行
`sudo apt update && sudo apt install -y python3-venv`。

```sh
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install .
hihost --version
hihost --help
```

新开终端后重新激活环境。集成到已有程序时，激活该程序的环境并运行
`python -m pip install "/path/to/products/python"`，IDE 也选择同一环境。
更新 SDK 源码后重新安装。

`hihost --help` 的开头显示 `hihost 0.1.0`。如果终端找不到 `hihost`，
可在同一环境中使用 `python -m hipnuc`，后面的参数相同。

## 读取、录制与发送指令

连接设备，关闭其它占用串口的程序；虚拟机需将 USB 转接器连接到虚拟机系统。

```sh
hihost list
hihost read
hihost read --duration 60 --record samples.jsonl
```

先按 Ctrl-C 停止连续读取，再尝试下一条命令。
`list` 显示 USB 串口和其它串口的数量，`hihost list --all` 展开全部端口，
`hihost list --json` 始终返回全部端口。自动发现搜索 USB 串口，最多 30 秒并显示进度。
连接多台设备、板载/GPIO UART 或其它端口时，用 `-p PORT` 指定；
知道速度时再加 `-b BAUD`：

```sh
hihost read -p COM3 -b 115200
hihost info -p COM3 -b 115200
hihost command "LOG VERSION" -p COM3 -b 115200
hihost command --file commands.txt -p COM3 -b 115200 --save
```

将 `COM3` 替换成实际端口，例如 Linux 的 `/dev/ttyUSB0`。
连接参数放在**最终命令之后**，包括 `modbus read`。
`-b` 只设置电脑连接速度，`hihost baudrate NEW_BAUD -p PORT` 修改当前连接所用设备端口的
波特率，并按新速度重新连接。
未知波特率用 `hihost scan -p PORT`，重启用 `reboot`，具体选项在最终命令后加
`--help` 查看。配置命令要求明确指定端口。

指令文件每行一条产品指令，`#` 和 `;` 开始注释。遇到失败停止执行；
`--save` 在全部成功后保存一次，设置需要重启时再加 `--reboot`。
连接变化使用 SDK 管理的 `baudrate` 和 `reboot` 命令。
指令名称及适用范围以产品的指令与编程手册为准。

`LOG HI91 ONMARK ONCE`（也支持 HI81/HI83/GGA/RMC）没有命令 ACK，SDK 等待对应类型的
样本。该报文可能本就在周期输出，因此收到样本不证明命令执行成功。
显式指定目标的命令，如 `LOG COM2 HI91 ONMARK ONCE`，需使用 `--no-reply`
（Python：`response="none"`）。

MATLAB 可读取 **HI91** JSONL 录制，见 [matlab/](../matlab/README_zh.md)。
录制保留全部已解码样本，与屏幕每种报文每秒最多五次的显示限速独立。
加 `--record-raw capture.bin` 保存原始接收字节，`--quiet` 隐藏读数，
`--jsonl` 输出机器数据。文件默认位于当前目录，明确加 `--overwrite` 才覆盖已有文件。
有限采集会完成当前接收批次。诊断输出到 stderr；
退出码：0 成功、1 失败、2 参数错误、130 Ctrl-C。

## Python API 与例程

```python
from hipnuc import SerialDevice

with SerialDevice() as device:
    for sample in device.iter_samples():
        print(sample.acceleration_m_s2, sample.angular_velocity_rad_s)
```

导入和构造对象不执行 I/O；调用是同步的，使用 `with` 打开和关闭资源。
`SerialDevice("COM3", baudrate=115200)` 指定已知连接，省略的连接参数通过发现流程补齐。

| 接口 | 用途 |
| --- | --- |
| `SerialDevice(..., timeout=2.0)` | 串口连接，超时单位为秒 |
| `device.read(timeout=None)`、`device.iter_samples(idle_timeout=None)` | 读取队列中或新收到的样本，空闲超时抛出 `ResponseTimeout`，默认使用设备 timeout |
| `device.read_info()` | 型号、固件及序列号 |
| `device.command("LOG VERSION").text` | 发送 ASCII 并获取回复；不支持的指令可能超时 |
| `device.save_config()`、`device.set_baudrate(baudrate)`、`device.reboot()` | 显式保存及当前串口连接上的受管理操作 |
| `Decoder().feed(data)` | 不依赖设备的增量字节解码；持续复用一个解码器，输入结束时调用 `finish()` |
| `sample.values`、`sample.to_dict()` | 协议特有字段及 JSON 兼容输出 |

每个样本对应单条报文，不拼接历史数据。API/JSON 使用 m/s²、rad/s、rad、T、Pa；
经纬度为度，温度为 °C；屏幕角度和角速度显示为度和度/秒。缺失值保持 `None`。
`quaternion_wxyz` 顺序为 WXYZ、机体到导航系；`euler_rad` 遵循设备配置的欧拉角约定。
heading 从北顺时针计算，与 Euler yaw 区分；SDK 不修改设备坐标配置。
`received_time_ns` 为主机接收时间，设备时间与 UTC 为独立字段。
`complete`、`issues`、`metadata` 保留解析和来源信息。
受管理的重启在等待设备恢复前清空旧样本队列；已经交给录制回调的样本保留。

通信异常继承 `HipnucError`：`TransportError`、`ResponseTimeout`、`DeviceError`、
`VerificationError`（读回不一致）。无效参数抛出 `ValueError`，录制 I/O 错误抛出 `OSError`。
`command()` 返回 `CommandResult`，包含 `command`、`text` 和 `acknowledged`；
ACK 不代表配置读回验证。仅发送，或 ONCE 收到样本但没有 ACK 时，返回
`acknowledged=False`；等待的回复未收到时，抛出 `ResponseTimeout`。

串口命令使用 `ResponseTimeout` 的子类 `CommandTimeout`：`sent=False` 表示命令
未写出，`sent=True` 表示是否执行尚未确认。遇到损坏输入时，SDK 在命令超时内等待
合法帧恢复同步；仍无法恢复时，仅重新打开一次原端口和波特率，供下一次调用使用。
重开失败则保持关闭。原命令不会自动重发，`response="none"` 仍为仅发送。

录制时先连接，再创建文件；开始读取前设置回调：

```python
from hipnuc import Recorder, SerialDevice

with (
    SerialDevice("COM3", baudrate=115200) as device,
    Recorder("samples.jsonl", raw_path="capture.bin") as recording,
):
    device.sample_sink = recording.write
    device.raw_sink = recording.write_raw
    for sample in device.iter_samples():
        pass
```

`Recorder` 写入 JSONL 和可选原始接收字节，`write_raw` 接收实际字节块，
不要传入 `sample.raw`。回调同步执行，应保持简短，不在回调中调用设备 I/O。
持续写入时每秒刷新，关闭时也会刷新；提供 `flush()`、`samples_written`、
`raw_bytes_written`。发现设备的通信发生在录制之前。
录制例程另外处理了 Ctrl-C，停止前会完成当前接收批次。

修改下方脚本顶部参数后执行 `python examples/read_samples.py`（或所选文件名）。
例程不接受命令行参数。

| 例程 | 用途 |
| --- | --- |
| [read_samples.py](examples/read_samples.py) | 读取测量；`PORT = None`、`BAUDRATE = None` 自动寻找 USB 连接 |
| [record_samples.py](examples/record_samples.py) | JSONL 录制；设置 `RAW_PATH` 可保留原始字节 |
| [send_commands.py](examples/send_commands.py) | 编辑 `COMMANDS`；默认查询身份/配置，不保存 |
| [modbus_multinode.py](examples/modbus_multinode.py) | 同一总线多个站号，顺序轮询 |

## Modbus RTU

明确指定端口和站号。默认 115200、8N1、站号 80，单播站号范围 1–247。
多机合并总线前，逐台准备 RTU 端口、输出方式、波特率及唯一站号；
多站 Modbus 总线上不要运行 ASCII `scan` 或 `command`。
需要关闭主动输出时，先在单机 ASCII 会话中发送 `LOG DISABLE`。
`LOG DISABLE` 只临时停发；重启后仍需保持 RTU 时，应关闭定时输出消息并显式保存该配置。

```sh
hihost modbus info -p COM3 --id 80
hihost modbus read -p COM3 --id 80
hihost modbus read -p COM3 --id 80 --duration 60 --record samples.jsonl
```

`read` 持续到 Ctrl-C、`--duration` 或 `--count`，`--interval` 设置轮询间隔。
寄存器地址、取值和可用性以对应型号的指令与编程手册为准。
原始寄存器访问用 `registers` / `write-register`，通信参数变化用
`set-id`、`baudrate`、`reboot`；在最终命令后加 `--help` 查看参数。

```python
from hipnuc import ModbusBus

with ModbusBus("COM3") as bus:
    device = bus.device(80)
    print(device.read_info())
    sample = device.read_sample()
    print(sample.acceleration_m_s2)
```

一条物理总线创建一个 `ModbusBus`，各站共用事务锁。FC03 一次读取 1–125 个寄存器，
FC06 写单个 16 位寄存器；宽数值为大端、高字在前。写入不自动重试，默认读回验证；
保存需显式使用 `--save` 或 `save_config()`。SDK 不保证同一测量块的所有寄存器来自同一固件周期。
JSONL 保留 `metadata.device_id`，不提供 Modbus 总线原始帧录制。
Python 录制时先打开总线，再打开 `Recorder`，写入每次返回的样本。
多站轮询见 [modbus_multinode.py](examples/modbus_multinode.py)。

## CAN

CAN 命令行需要 Linux SocketCAN。在本目录安装可选依赖，并按设备波特率配置接口：

```sh
python -m pip install ".[can]"
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
hihost can read -i can0
hihost can read -i can0 --id 8 --record samples.jsonl
```

使用 CAN FD 时先配置仲裁段／数据段波特率，再给 `can read` 加上 `--fd`。
`--id` 筛选一个来源，省略则接收所有来源。录制沿用串口的 JSONL 格式和
文件保护；每条记录保留 `node_id`、CAN 标识符和主机接收时间，不合并不同 PGN。
接口状态和原始抓包使用 `ip`、`candump` 等系统工具。

原始寄存器操作必须指定目标：`hihost can reg read ADDRESS -i can0 --id 8`
或 `hihost can reg write ADDRESS VALUE -i can0 --id 8`。地址和值支持十进制或 `0x`。
写入只检查回复，不自动保存、重启，也不证明配置已经生效。不支持的请求可能超时。

集成时直接使用标准 `python-can` 总线和 SDK 函数：

```python
import can
from hipnuc.can import decode_message

with can.Bus(interface="socketcan", channel="can0", ignore_config=True) as bus:
    for message in bus:
        sample = decode_message(message)
        if sample is not None:
            print(sample.values)
```

使用其他适配器（包括 Windows）时，按后端要求创建总线，例如
`can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=500000, ignore_config=True)`，
再使用同一套 SDK 函数。CAN FD 的 timing 参数由适配器决定。

`decode_message()` 对无关报文返回 `None`，对格式错误的已支持报文抛出 `ValueError`。
`read_register(bus, node_id, address)` 返回原始整数；
`write_register(bus, node_id, address, value)` 检查写入回显，默认超时均为 2 秒。
每条总线只由一个接收者使用；寄存器事务会消费期间收到的其他报文，不能与读取并行。
`make_trigger(node_id, pgn)` 生成报文，交给 `bus.send()` 或 `bus.send_periodic()`
发送，不等待确认回复。

## 固件升级

桌面操作可使用 CHCenter。终端或无桌面环境使用**对应设备型号**的应用固件，
并明确指定目标：

```sh
hihost update firmware.hex -p /dev/ttyUSB0 -b 115200
hihost can update firmware.hex -i can0 --id 8
```

Windows 将端口换成 `COM3` 或实际端口。CAN 升级需要可选 CAN 依赖，节点 ID
为 1–127；`--bin` 显式选择原始二进制文件。升级前关闭其他读取程序并停止周期发送。
bootloader 无法验证固件型号。传输和启动请求成功不代表新应用已经运行；
完成后用 `info` 重新查询或读取测量。失败或 Ctrl-C 后不自动重启设备。

程序集成可用 `update_serial(path, port=..., baudrate=...)`，由函数打开和关闭连接；
`update_can(bus, node_id, path)` 使用调用方持有的总线。二者返回 `UpdateResult`，
给出传输字节数与引导程序确认标志。可选的 `progress(written, total)`
回调同步运行，抛出异常即取消升级。串口升级不需要 CAN 依赖。

## 常见问题

| 现象 | 处理方式 |
| --- | --- |
| 缺少 Python / venv / pip | 使用 Python 3.10+。支持 Ubuntu 22.04 默认的 3.10，不支持 20.04 默认的 3.8；Ubuntu/Debian/Pi OS 安装 `python3-venv`。 |
| APT 等待锁 | 等系统更新完成，不删除锁或强杀更新程序。 |
| `UNKNOWN-0.0.0` / `No module named hipnuc` | 激活正确环境、升级 pip，再从当前 `python/` 目录重新安装。 |
| PowerShell 阻止激活 | 用 `.\.venv\Scripts\python.exe` 安装依赖，直接运行 `.\.venv\Scripts\hihost.exe`。 |
| 没有串口 / 串口占用 | 检查虚拟机 USB 透传、线缆/驱动和其它串口程序；非 USB 端口用 `hihost list --all`。 |
| 串口打开但无有效样本 | 检查实际波特率、接线、输出方式和频率，低频输出需足够的 `--timeout`。 |
| 下载 / 证书错误 | 检查网络、时间及所需代理/证书设置，不关闭 TLS 校验。 |

HGFS 等共享目录不能创建 venv 符号链接时，将环境放在本地磁盘；
源码完整且目录可写时可以继续留在共享目录：

```sh
python3 -m venv "$HOME/.venvs/hipnuc"
source "$HOME/.venvs/hipnuc/bin/activate"
python -m pip install --upgrade pip
python -m pip install .
```

新终端用同一条 `source` 命令激活这个外置环境。
Linux 串口权限先看 `ls -l /dev/ttyUSB0` 和 `id -nG`；设备组为 `dialout` 时，
执行 `sudo usermod -a -G dialout "$(id -un)"`，然后退出登录再登录或重启。
虚拟环境与串口权限无关。
