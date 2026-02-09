# Python 数据读取即模块配置例程 #

## 简介

本示例包提供了如何使用 Python 读取 HiPNUC 模块数据或发送指令给模块的示例，支持 HI91/HI81/HI83 与 NMEA 输出


## 测试平台

- Windows 11
- Ubuntu 20.04
- 树莓派 4B

## 目录结构

├── main.py: 主程序入口
├── README.md
├── requirements.txt
├── commands: 命令脚本
└── parsers: 解析器脚本

## 安装与使用

### 1. 安装依赖

首先，确保你已经安装了 Python 3.6 及以上版本。然后，使用以下命令安装所需依赖：

```python
$ pip install -r requirements.txt
```

### main.py --help: 显示帮助

`main.py --help` : 显示例程读取脚本help文档:

```
$ python main.py --help
Usage: main.py [OPTIONS] COMMAND [ARGS]...

  HiPNUC Python Example

Options:
  --help  Show this message and exit.

Commands:
  list   List all available serial ports
  probe  Auto probe device port and baudrate
  read   Read data from the specified serial port
  write  Send command(s) to the module
  send   Send command(s) to the module
```

### main.py list: 列出设备可用串口

```
$ python main.py list
Your Python version is 3.11.2.
Found 3 available serial port(s):
Device: COM13
Device: COM20
Device: COM27
```

如需查看额外信息：

```
python main.py list --details
```

### main.py read: 读取数据

* 帮助: `main.py read --help`

* 示例(Linux): `$ python main.py read -p /dev/ttyUSB0 -b 115200`

* 示例(Windows): `$ python main.py read -p COM3 -b 115200`

返回模块数据示例（屏幕输出为 JSON 行）：

```
{"type":"HI91","main_status":5384,"temperature":35,"air_pressure":100676.0,"system_time":1840392,"acc":[0.0,0.0,9.81],"gyr":[-0.06,0.05,-0.03],"mag":[21.06,6.18,-34.02],"roll":-0.51,"pitch":0.58,"yaw":3.86,"quat":[0.999,0.005,-0.004,0.034]}
{"type":"status","frame_rate_hz":101}
```

可选记录功能：

* 原始数据记录：`python main.py read -p COM3 -b 115200 -r imu_raw.bin`
* JSON 记录：`python main.py read -p COM3 -b 115200 -j imu_data.json`
* 同时记录：`python main.py read -p COM3 -b 115200 -r imu_raw.bin -j imu_data.json`

JSON 文件为逐行 JSON（JSONL），包含 HI91/HI81/HI83 与 NMEA 数据。

### main.py probe: 自动探测设备

```
$ python main.py probe
Probing COM27 @ 115200 ...
========== Device Found ==========
Port: COM27
Baud Rate: 115200
Device Info:
PNAME=HI14
BUILD=Jul 31 2024
UUID=048495968DD31708
APP_VER=154
BL_VER=108
OK
==================================
```

只探测指定端口：

```
python main.py probe --ports COM27
```

### main.py write: 发送指令到模块

发送一条配置指令(以发送“LOG VERSION”为例):

* 帮助: `main.py write --help`

* 示例(Linux): `$ python main.py write -p /dev/ttyUSB0 -b 115200 "LOG VERSION"` 

* 示例(Windows): `$ python main.py write -p COM3 -b 115200 "LOG VERSION"`

```
$ python main.py write -p COM27 -b 115200 "LOG VERSION"
Your Python version is 3.11.2.
Serial port COM27 opened with baud rate 115200
PNAME=HI14
BUILD=Jul 31 2024
UUID=048495968DD31708
APP_VER=154
BL_VER=108
OK
```

> 1. 所有配置指令请参考用户编程手册
> 2. write/send 会先尝试停止模块输出，再发送命令，最后恢复输出
> 3. 需要保存配置时请手动发送 `SAVECONFIG`

批量配置文件示例：

```
python main.py write -p COM3 -b 115200 commands.txt
```

## 常见问题

### 如何更改串口配置？

你可以通过命令行参数 `-p` 和 `-b` 来指定串口端口和波特率。例如：

```bash
$ python main.py read -p COM3 -b 115200
```

### 如果遇到权限问题怎么办？

在 Linux 系统上，访问串口设备可能需要超级用户权限。你可以使用 `sudo` 命令来运行脚本：

```bash
$ sudo python main.py read -p /dev/ttyUSB0 -b 115200
```

