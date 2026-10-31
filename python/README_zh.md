# HiPNUC Python SDK

[English](README.md) | [中文](README_zh.md)

`hipnuc` 是 HiPNUC IMU、AHRS、MRU 和 INS 模块的 Python SDK，`hihost` 是配套的命令行工具。
通过串口、Modbus RTU 或 CAN，可以读取和录制数据、发送命令、修改设置、升级固件。

- 支持 Python 3.10–3.14，Windows、Linux（含树莓派系统等嵌入式板卡）和 macOS
- 串口报文：HI91、HI81、HI83、NMEA GGA 和 RMC
- 可选接口：Modbus RTU（`[modbus]`，PyModbus）、CAN J1939 / CANFD83（`[can]`，python-can）
- 支持设备：HI01–HI06、HI12–HI18、HI70/HI71、CH0X0；`-I2C` 型号没有 CAN

## 安装

在本仓库的 `python/` 目录打开终端。

Windows（PowerShell）：

```powershell
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install .
hihost --version
```

Linux 和 macOS：

```sh
python3 -m venv .venv            # Ubuntu/Debian/Pi OS：先执行 sudo apt install python3-venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install .
hihost --version
```

需要可选接口时执行 `python -m pip install ".[modbus,can]"`。
每次新开终端都要重新激活环境。在已有项目中使用 SDK 时，先激活该项目的环境，再执行
`python -m pip install /path/to/products/python`；SDK 源码更新后重新安装一次。
终端找不到 `hihost` 时，改用 `python -m hipnuc`，参数不变。

板卡不能联网，或者系统 Python 低于 3.10，见[嵌入式 Linux 板卡](#嵌入式-linux-板卡)。

## 快速上手

连接设备，关闭其他占用该串口的程序。

```sh
hihost list                                       # 列出 USB 串口
hihost read                                       # 自动查找设备并显示读数，Ctrl-C 停止
hihost read --duration 60 --record samples.jsonl  # 录制 60 秒
hihost info                                       # 型号、固件版本、序列号
```

不加 `-p` 时，`hihost` 会向每个 USB 串口发送身份查询来查找设备，最长需要 30 秒。
接了多台设备或使用非 USB 串口时，用 `-p` 指定端口；知道波特率就再加 `-b`：

```sh
hihost read -p COM3 -b 115200
hihost read -p /dev/ttyUSB0 -b 115200
hihost read -p /dev/serial/by-id/usb-...   # 板卡上有多个 USB 串口设备时，用固定名称
```

`hihost list --all` 会同时列出非 USB 串口。Linux 上部分板载 UART（如 `/dev/ttyS3`）不在列表中，
直接用 `-p` 指定即可。选项写在最后一个命令词之后，例如 `hihost modbus read -p COM3`；
`hihost <命令> --help` 列出全部选项。

### 命令与设置

```sh
hihost command "LOG VERSION" -p COM3
hihost command --file commands.txt -p COM3 --save   # 每行一条命令
hihost baudrate 921600 -p COM3                      # 修改设备波特率
hihost scan -p COM3                                 # 查找该端口上设备的波特率
hihost reboot -p COM3
```

命令名称和作用见产品的指令与编程手册。命令文件中 `#` 和 `;` 开始注释，遇到失败即停止；
加 `--save` 时，全部成功后保存一次。`-b` 只设置电脑端的波特率，修改设备波特率要用
`hihost baudrate`，它会按新波特率重新连接。`LOG <报文> ONMARK ONCE` 没有应答，
要单独发送并加 `--no-reply`（Python 中为 `response="none"`）。

### 录制

`--record` 把每个解码后的样本写成 JSON Lines，采用国际单位（角度为弧度）。
屏幕上的角度以度显示，每种报文每秒最多显示五次，录制则保留全部样本。

- `--record-raw capture.bin` 同时保存收到的原始字节，之后可用 `hihost decode capture.bin`
  离线解码（加 `--jsonl` 或 `--record out.jsonl`）。
- `--quiet` 不显示读数，`--jsonl` 每个样本输出一行 JSON。
- 已有文件不会被覆盖，除非加 `--overwrite`。

停止时 `hihost` 会显示每种报文的实测速率。退出码：0 成功，1 失败，2 参数错误，130 Ctrl-C。
`hihost -v <命令>` 把 SDK 调试日志输出到 stderr。

环境变量 `HIHOST_PORT`、`HIHOST_BAUDRATE`、`HIHOST_MODBUS_ID`、`HIHOST_CAN_INTERFACE`、
`HIHOST_CAN_ID` 和 `HIHOST_CAN_BITRATE` 为对应选项提供默认值。启用 shell 补全：
`eval "$(_HIHOST_COMPLETE=bash_source hihost)"`（或 `zsh_source`、`fish_source`）。

## Python

```python
from hipnuc import SerialDevice

with SerialDevice() as device:  # 或 SerialDevice("COM3", baudrate=115200)
    for sample in device.iter_samples():
        print(sample.acceleration_m_s2, sample.angular_velocity_rad_s)
```

创建对象时不做任何 I/O；`with` 负责打开和关闭端口，所有调用都是同步的。
端口或波特率为 `None` 时自动查找。

| 接口 | 用途 |
| --- | --- |
| `SerialDevice(port=None, baudrate=None, timeout=2.0)` | 串口连接，时间单位为秒 |
| `device.read()`、`device.iter_samples()` | 读取下一个样本或连续读取；链路一直无数据时抛出 `ResponseTimeout` |
| `device.read_info()` | 型号、固件版本、序列号 |
| `device.command("LOG VERSION")` | 发送 ASCII 命令，返回含 `text` 和 `acknowledged` 的 `CommandResult` |
| `device.save_config()`、`device.set_baudrate(rate)`、`device.reboot()` | 保存、修改波特率、重启 |
| `Decoder().feed(data)` | 解码任意来源的字节，输入结束时调用 `finish()` |
| `sample.values`、`sample.to_dict()` | 报文字段；可直接转成 JSON 的字典 |

**单位与约定。** 每个样本对应一条报文。数值采用国际单位（m/s²、rad/s、rad、T、Pa），
经纬度为度，温度为 °C，缺失值为 `None`。`quaternion_wxyz` 表示机体系到导航系的旋转，
`euler_rad` 沿用设备配置的欧拉角约定。航向（heading）从北向顺时针计量，与欧拉角的 yaw 不同。
`received_time_ns` 是主机收到该数据块的时间；样本的时间请用设备时间字段。

**异常。** 通信异常都继承自 `HipnucError`：`TransportError`、`ResponseTimeout`、
`DeviceError`（请求被拒绝）和 `VerificationError`（读回不一致）。参数无效时抛出 `ValueError`。
命令不会自动重发。命令发出后超时，`CommandTimeout.sent` 为真，连接随之关闭，
但设备可能仍在执行这条命令，等它完成后再重新连接。

在 Python 中录制时，先打开设备，再创建文件并设置回调：

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

`write_raw` 接收的是原始数据块，不要传入 `sample.raw`。回调在读取线程中执行，
应尽量简短，也不要在回调里访问设备。文件每秒刷新一次，关闭时也会刷新。

### 例程

修改脚本开头的常量后直接运行，例如 `python examples/read_samples.py`。

| 例程 | 功能 |
| --- | --- |
| [read_samples.py](examples/read_samples.py) | 显示测量值；`PORT` 为 `None` 时自动查找 USB 设备 |
| [record_samples.py](examples/record_samples.py) | 录制 JSON Lines，设置 `RAW_PATH` 时同时保存原始字节 |
| [send_commands.py](examples/send_commands.py) | 依次发送 `COMMANDS` 中的命令 |
| [modbus_multinode.py](examples/modbus_multinode.py) | 在一条总线上轮询多个 Modbus 站 |
| [can_read.py](examples/can_read.py) | 通过任意 python-can 总线读取 CAN 测量值和身份 |
| [update_firmware.py](examples/update_firmware.py) | 通过串口或 CAN 升级固件 |

## Modbus RTU

```sh
python -m pip install ".[modbus]"
hihost modbus info -p COM3 --id 80
hihost modbus read -p COM3 --id 80 --duration 60 --record samples.jsonl
```

出厂设置为 115200、8N1、站号 80，单播站号范围 1–247。设备接入总线前，逐台做好准备：

- 关闭 Modbus 端口上的周期输出，否则数据帧会和应答混在一起。单独连接这台设备，
  用 ASCII 命令逐一关闭定时报文（例如 `LOG HI91 ONTIME 0`）并保存。
  `LOG DISABLE` 只在下次重启前有效。
- 给每台设备分配不同的站号。共享的 Modbus 总线上不要运行 ASCII 的 `scan` 或 `command`。

`read` 一直轮询，直到 Ctrl-C、达到 `--duration` 或 `--count`；`--interval` 设置两次轮询的间隔。
`registers` 和 `write-register` 直接读写寄存器（FC03/FC06），`set-id` 和 `baudrate`
修改通信参数，`reboot` 重启设备。寄存器地址见产品手册。

```python
from hipnuc import ModbusBus

with ModbusBus("COM3") as bus:
    device = bus.device(80)
    print(device.read_info())
    print(device.read_sample().acceleration_m_s2)
```

一个 `ModbusBus` 独占一个端口，总线上的各台设备轮流使用。超过 16 位的数值按大端存放，高字在前。
写入后默认读回校验，只有明确要求（`--save` 或 `save_config()`）时才保存。

## CAN

### Linux（SocketCAN）

```sh
python -m pip install ".[can]"
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
# CAN FD 产品：... bitrate 500000 dbitrate 4000000 fd on restart-ms 100
sudo ip link set can0 up
hihost can scan -i can0 --identify
hihost can read -i can0 --id 8 --record samples.jsonl
```

出厂设置为 500 kbit/s、节点号 8；设置 `restart-ms` 后，控制器进入 bus-off 会自行恢复。
PEAK 适配器使用内核自带的 `peak_usb` 驱动；PEAK 自己的 Linux 驱动会替换它，默认编译的版本不提供 SocketCAN。
命令失败时，`hihost` 会指出可能的原因，并给出修正用的 `ip link` 命令。
要让设置在重启后保留，把它写进系统的网络配置，例如 `/etc/systemd/network/80-can0.network`：

```ini
[Match]
Name=can0

[CAN]
BitRate=500000
RestartSec=100ms
```

### Windows（PCAN）

安装 PEAK 驱动（含 PCAN-Basic）。PCAN 通道还需要给出节点的波特率，所以下文示例中的 `-i can0`
要换成 `-i PCAN_USBBUS1 --bitrate 500000`，也可以一次性设置 `HIHOST_CAN_INTERFACE` 和
`HIHOST_CAN_BITRATE`。PCAN 通道以 Classic CAN 模式打开。

```sh
hihost can scan -i PCAN_USBBUS1 --bitrate 500000 --identify
```

### 读取与配置

`scan` 列出总线上的节点及每个 PGN 的速率，加 `--identify` 再读取身份。`read --id` 只接收一个节点，
接收 CAN FD 帧要加 `--fd`。录制格式与串口相同，每条记录保留节点号和 CAN 标识符。

配置命令针对单个节点（`--id`，1–127，85 留给主机），设备在写入时即保存。修改节点号或波特率后，重启才生效：

```sh
hihost can info -i can0 --id 8
hihost can period acc 10 -i can0 --id 8          # 周期，单位 ms；也可为 off 或 sync（SYNC_IN 触发）
hihost can output off -i can0 --id 8             # 关闭全部周期 PGN
hihost can trigger quat -i can0 --all            # 每个节点发一帧，无应答
hihost can set-id 9 -i can0 --id 8 --reboot
hihost can bitrate 1000000 -i can0 --id 8 --reboot   # 随后把 can0 改为 1000000
hihost can reboot -i can0 --id 8
```

PGN 名称：acc、gyr、mag、roll_pitch、yaw、quat、temp、inclination、time、position、altitude、
gnss_status、velocity、canfd83，也可以写编号。执行 `can reboot --factory-reset` 后，
节点可能以 500 kbit/s、节点号 8 重新上线。

直接读写寄存器：`hihost can reg read ADDRESS -i can0 --id 8` 和
`hihost can reg write ADDRESS VALUE -i can0 --id 8`（十进制或 `0x`）。写入只校验回显。
CAN FD 输出由两个寄存器设置：`0x180` 是 CANFD83 的字段位图（例如 `0x12B`），
`0x9B` 是数据段波特率编码（`11` 表示 4 Mbit/s）。逐个写入，然后重启。

### Python

使用 python-can 总线和 SDK 函数：

```python
import can
from hipnuc.can import decode_message

with can.Bus(interface="socketcan", channel="can0", ignore_config=True) as bus:
    for message in bus:
        try:
            sample = decode_message(message)
        except ValueError:  # Another device's frame on a shared PGN.
            continue
        if sample is not None:
            print(sample.values)
```

其他适配器的用法相同，按其要求创建总线即可，例如
`can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=500000, ignore_config=True)`。
CANFD83 帧只在 FD 总线上收到：先把接口配置为 CAN FD，再用 `fd=True` 创建总线。

`decode_message()` 遇到无关报文返回 `None`，遇到格式错误的 HiPNUC 报文抛出 `ValueError`。
适配器的时间戳保存在 `metadata["can_timestamp_s"]`，`received_time_ns` 留给调用方填写。
`read_register` 和 `write_register` 直接读写寄存器；`read_info`、`reboot`、`request_reboot`、
`wait_for_node`、`factory_reset`、`save_config`、`set_node_id`、`set_bitrate`、`set_pgn_period`、
`set_output_enabled` 和 `send_trigger` 提供常用操作，参数为调用方的总线和节点号；
`scan(bus, duration)` 自行查找节点。

每条总线只能有一个接收者：寄存器函数等待应答时，会取走期间收到的其他帧。
接收过滤必须放行节点的应答：寄存器用扩展帧 `0x0CEF55xx`，升级用标准帧 `0x580` + 节点号。
`make_trigger(node_id, pgn)` 生成触发帧，交给 `bus.send()` 或 `bus.send_periodic()` 发送。
`pgn` 是编号，例如 `hipnuc.can` 中的 `PGN_NAMES["QUAT"]`；节点号 255 表示全部节点，不会有应答。

## 固件升级

```sh
hihost update firmware.hex -p /dev/ttyUSB0
hihost can update firmware.hex -i can0 --id 8
hihost can update firmware.hex -i PCAN_USBBUS1 --bitrate 500000 --id 8   # Windows
```

请使用 HiPNUC 为该型号提供的 Intel HEX 应用固件，bootloader 无法区分产品。`hihost` 在操作设备之前先检查镜像，
省略 `-b` 时自动识别串口波特率，完成后确认新固件已经运行（`--no-verify` 跳过这一步）。
任何版本都可以安装，包括较旧的版本。升级前关闭占用该端口或总线的其他程序；串口升级期间保持设备静止。
190 KB 左右的固件，串口升级约 25 秒，CAN 500 kbit/s 约 40 秒。也可以用 HiPNUC 的上位机 CHCenter 升级。

升级中途停止时，重新执行同一条命令即可。设备完全没有应答时（例如刷入了其他产品的固件），
命令会提示给设备重新上电，并在设备启动时接管 bootloader：

- 串口：拔下设备的整个连接器（电源线和信号线一起），再插回。USB 串口适配器保持插在电脑上。
- CAN：bootloader 以 500 kbit/s、节点号 8 启动，请把接口设为 500 kbit/s 并使用 `--id 8`。
  升级期间该节点号上只能有一台设备。

在 Python 中，`update_serial(path, port=..., baudrate=None)` 自行打开和关闭连接，
`update_can(bus, node_id, path)` 使用调用方已按节点波特率打开的总线。两者都返回
`UpdateResult(bytes_written, image_version, previous_version, verified_version)`，
未知的版本为 `None`；都支持 `verify=False` 和可选的 `progress(written, total)` 回调，
在回调中抛出异常即可取消升级。设备没有应答时，重新上电的提示以 WARNING 级别写入
`hipnuc.update` 日志，调用最多等待 `power_cycle_wait` 秒（默认 60，设为 `0` 立即失败）。
镜像文件无效时抛出 `ValueError`，文件无法读取时抛出 `OSError`，bootloader 拒绝镜像时抛出 `DeviceError`。
串口升级不需要安装 CAN 依赖。

## 嵌入式 Linux 板卡

SDK 及其依赖都通过 wheel 安装，x86、64 位 ARM（例如 RK3588）和 32 位 ARM 板卡都不需要编译器。

**无网络。** 在能联网的电脑上构建 SDK 的 wheel，并按板卡的 Python 版本下载依赖。
平台参数：64 位 ARM 用 `manylinux2014_aarch64`，32 位 ARM 用 `linux_armv7l`，
x86 用 `manylinux2014_x86_64`。把 `wheels/` 拷到板卡，在虚拟环境中安装
（Ubuntu/Debian 板卡需要 `python3-venv`）。`--force-reinstall` 也会替换已安装的同版本 SDK。

```sh
python -m pip wheel --no-deps -w wheels .
python -m pip download -d wheels --only-binary=:all: --platform manylinux2014_aarch64 --python-version 3.10 "pyserial>=3.5,<4" "click>=8.1,<9" "python-can>=4.6,<5" "pymodbus>=3.15,<3.16"
# 在板卡上，激活虚拟环境后：
python -m pip install --no-index --find-links wheels --force-reinstall "hipnuc-sdk[can,modbus]"
```

**Python 低于 3.10**（Ubuntu 20.04 自带 3.8，Debian 11 自带 3.9）。改用 GitHub 上
python-build-standalone 发布的独立版 CPython：不需要 root，可与系统 Python 并存，要求 glibc 2.17 及以上。
下载 `cpython-3.10.<x>+<date>-aarch64-unknown-linux-gnu-install_only.tar.gz`
（32 位 ARM 为 `armv7-unknown-linux-gnueabihf`；中国大陆可用 npmmirror 二进制镜像，文件相同），
用 `--python-version 3.10` 下载 wheel，一起拷到板卡上执行：

```sh
tar -xzf cpython-3.10.*-install_only.tar.gz -C "$HOME"
"$HOME/python/bin/python3" -m venv "$HOME/.venvs/hipnuc"
. "$HOME/.venvs/hipnuc/bin/activate"
python -m pip install --no-index --find-links wheels --force-reinstall "hipnuc-sdk[can,modbus]"
```

## 常见问题

| 现象 | 处理方法 |
| --- | --- |
| `python3 -m venv` 失败 | 安装 `python3-venv`（Ubuntu/Debian/Pi OS）。 |
| Python 低于 3.10 | 使用独立版 Python，见[嵌入式 Linux 板卡](#嵌入式-linux-板卡)。 |
| PowerShell 禁止运行 `Activate.ps1` | 直接运行 `.\.venv\Scripts\python.exe -m pip ...` 和 `.\.venv\Scripts\hihost.exe`。 |
| `No module named hipnuc` 或 `UNKNOWN-0.0.0` | 激活正确的环境，升级 pip，再在本目录重新安装。 |
| `Modbus support requires` / `CAN support requires` | 在同一环境安装对应依赖：`python -m pip install ".[modbus]"` 或 `".[can]"`。 |
| Linux：打开端口时权限不足 | 把当前用户加入端口所属的组，通常是 `dialout`（`ls -l /dev/ttyUSB0` 可查看）：`sudo usermod -a -G dialout "$(id -un)"`，然后注销并重新登录。 |
| 找不到端口或端口被占用 | 检查线缆和驱动、虚拟机的 USB 直通，以及是否有其他程序占用端口。Linux 上新插入的端口可能被 ModemManager 占用几秒钟。 |
| 端口能打开但没有样本 | 检查波特率、接线以及输出是否开启；输出频率较低时加大 `--timeout`。 |
| CAN：没有节点应答或 bus-off | 检查波特率、接线和终端电阻（两端各 120 Ω）。波特率正确仍收不到任何帧时，拔插一次 USB CAN 适配器。 |
| 升级中途停止 | 重新执行同一条命令，见[固件升级](#固件升级)。 |
