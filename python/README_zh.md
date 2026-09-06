# HiPNUC Python SDK

[English](README.md) | [中文](README_zh.md)

在命令行或自己的 Python 程序中读取、配置和录制 HiPNUC IMU/AHRS/MRU 与 INS。
支持串口二进制/NMEA 和 Modbus RTU，适用于 Python 3.10–3.14、Windows、Linux
（含 Ubuntu 和树莓派系统）及 macOS。

## 安装

下载仓库，在 `python/` 目录打开终端（从仓库根目录执行 `cd python`）。
如果下载的是仓库 ZIP，请先解压。连接设备，然后创建虚拟环境：

**Windows PowerShell：**

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install .
.\.venv\Scripts\python.exe -m hipnuc list
.\.venv\Scripts\python.exe -m hipnuc read
```

**Linux / 树莓派 / macOS：**

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install .
python -m hipnuc list
python -m hipnuc read
```

`list` 列出系统串口。`read` 自动寻找 HiPNUC 设备和波特率，显示选中的连接后开始读数；
自动发现最多等待 30 秒，期间显示正在尝试的端口、波特率及结果。
发现多台设备时，用 `-p` 指定端口。按 Ctrl-C 停止。

Windows 命令直接使用虚拟环境中的 Python，无需激活环境或修改 PATH。
Linux/macOS 新开终端后，重新激活虚拟环境即可。

## 读取和录制

以下示例使用 `python` 简写。Windows PowerShell 中，请像上面一样换成
`.\.venv\Scripts\python.exe`。

```sh
# Show device identity.
python -m hipnuc info

# Use a known connection. Replace COM3 with the actual port on Linux/macOS.
python -m hipnuc read -p COM3 -b 115200

# Record 60 seconds of decoded samples.
python -m hipnuc read --duration 60 --record samples.jsonl

# Also keep the original received bytes.
python -m hipnuc read --record samples.jsonl --record-raw capture.bin
```

屏幕默认每种报文每秒最多显示 5 次，文件独立记录全部已解码样本。
`--quiet` 隐藏读数；`--jsonl` 将屏幕读数换成完整的 JSON 数据输出。
已有文件默认不覆盖；需要覆盖时加 `--overwrite`。

JSONL 和 Python API 使用 SI 单位：加速度 m/s²、角速度 rad/s、姿态角 rad。
屏幕上的角度和角速度使用更直观的 °、°/s。缺失测量不会补成零。

在 `hipnuc` 子命令后加 `--help` 查看参数。配置用法见 [API 与 CLI 参考](docs/api.md)，
有站号的 RTU 设备见 [Modbus 指南](docs/modbus.md)。

## 发送指令

```sh
python -m hipnuc command "LOG VERSION" -p COM3 -b 115200
python -m hipnuc command --file commands.txt -p COM3 -b 115200 --save
```

`commands.txt` 每行一条产品指令，每条执行后立即显示回复。`--save` 在全部成功后保存一次；
设置需要重启时再加 `--reboot`。遇到失败立即停止。

## 在自己的程序中使用

```python
from hipnuc import Recorder, SerialDevice

with SerialDevice() as device, Recorder("samples.jsonl") as recording:
    for sample in device.iter_samples():
        recording.write(sample)
        print(sample.acceleration_m_s2)
```

已知连接参数时使用 `SerialDevice("COM3", baudrate=115200)`。
`sample.to_dict()` 可转换成 JSON 兼容数据。解析已有字节直接使用 `Decoder.feed(data)`，
无需打开设备。

例程采用直接修改、运行的方式：先编辑脚本顶部的常量，再从 SDK 的 `python/` 目录执行，
例如 `python examples/read_samples.py`，不需要命令行参数。
Windows 中像上面一样使用 `.\.venv\Scripts\python.exe`。

| 例程 | 用途 |
| --- | --- |
| [read_samples.py](examples/read_samples.py) | 读取 IMU/INS 测量；`PORT = None`、`BAUDRATE = None` 自动寻找连接 |
| [record_samples.py](examples/record_samples.py) | 录制到 `JSONL_PATH = "samples.jsonl"`；设置 `RAW_PATH` 可同时保存原始接收字节 |
| [send_commands.py](examples/send_commands.py) | 编辑 `COMMANDS`；默认只查询版本和输出配置，不自动保存 |
| [modbus_multinode.py](examples/modbus_multinode.py) | 轮询站号 80、81；按总线修改 `PORT`、`BAUDRATE`、`NODE_IDS`、`INTERVAL_S` |

读取、录制和 Modbus 轮询持续运行，按 Ctrl-C 停止。录制不覆盖已有文件，再次录制时
请换一个输出文件名。

## 常见问题

- **安装成功但找不到命令：**使用同一个 Python 执行 `-m pip install` 和 `-m hipnuc`。
  上面的 Windows 命令不依赖 Scripts 目录是否在 PATH 中。
- **没有串口：**检查供电、USB 数据线和 USB 转串口驱动；Windows 下查看设备管理器
  中是否出现 COM 口。
- **连接或读取失败：**关闭占用串口的 CHCenter 等程序。知道端口和波特率时同时指定
  `-p`、`-b`；设备有合法测量输出时，即使身份查询没有回复，也可读取数据。
- **设备关闭了输出：**只接一台设备，以实际波特率运行
  `python -m hipnuc command "LOG ENABLE" -p COM3 -b 115200`。
- **数据不连续或校验错误：**确认输出频率与串口带宽匹配，必要时降低输出频率或提高设备波特率。
  `-b` 只设置电脑连接速度；`baudrate NEW_BAUD` 修改设备波特率。
- **不知道波特率：**运行 `python -m hipnuc scan -p COM3`。多站 Modbus 总线不要运行
  ASCII 发现命令，请使用 [Modbus 指南](docs/modbus.md)。
- **Linux 权限不足：**授予当前用户串口访问权限，常见设备组为 `dialout`，修改后重新登录。
  存在 `/dev/serial/by-id/...` 时优先使用该稳定路径。
- **Python/venv 不可用：**安装 Python 3.10 或以上版本。支持 Ubuntu 22.04 自带的
  Python 3.10；Ubuntu、Debian 和树莓派系统可能需要先运行
  `sudo apt install python3-venv`。SDK 安装在虚拟环境中，不要使用 `sudo pip`。
- **树莓派 GPIO UART：**在系统配置中启用 UART、关闭串口登录控制台；使用 USB 转串口
  不需要这项 GPIO 设置。

[IMU 指令与编程手册](https://download.hipnuc.com/products/imu/cum.html)
· [INS 指令与编程手册](https://download.hipnuc.com/products/ins/cum.html)
