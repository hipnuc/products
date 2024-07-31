# Python 数据读取即模块配置例程 #

## 简介

本示例包提供了如何使用 Python 读取 HiPNUC 模块数据或发送一条指令给模块的示例


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
  list  List all available serial ports
  read  Read data from the specified serial port
  send  Send a command to the module
```

### main.py list: 列出设备可用串口

```
$ python main.py list
Your Python version is 3.11.2.
Found 3 available serial port(s):
Device: COM13                Manufacturer: Microsoft            Permissions: Unknown
Device: COM20                Manufacturer: Microsoft            Permissions: Unknown
Device: COM27                Manufacturer: Silicon Labs         Permissions: Unknown

```

### main.py read: 读取数据

* 帮助: `main.py read --help`

* 示例(Linux): `$ python main.py read -p /dev/ttyUSB0 -b 115200`

* 示例(Windows): `$ python main.py read -p COM3 -b 115200`

返回模块数据：

  ```
  frame received: 10, frame_rate: 101Hz
  Temperature  :     37 C
  Pressure     : 99459.211 Pa
  Acceleration : X=    0.010, Y=    0.010, Z=    1.003 G
  Gyroscope    : X=   -0.076, Y=    0.053, Z=   -0.034 dps
  Magnetometer : X=   21.062, Y=    6.181, Z=  -34.019 uT
  Euler Angles : Roll=   -0.514, Pitch=    0.575, Yaw=    3.860 deg
  Quaternion   : W=    0.999, X=    0.005, Y=   -0.004, Z=    0.034
  Timestamp    : 146140 ms
  PPS Sync Time:   6876 ms
  ```

### main.py send: 发送一条指令到模块

发送一条配置指令(以发送“LOG VERSION”为例):

* 帮助: `main.py send --help`

* 示例(Linux): `$ python main.py send -p /dev/ttyUSB0 -b 115200 "LOG VERSION"` 

* 示例(Windows): `$ python main.py send -p COM3 -b 115200 "LOG VERSION"`

```
$ python main.py send -p COM27 -b 115200 "LOG VERSION"
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
> 2. 每次调用send命令，都会默认先尝试停止模块输出，然后发送用户命令字符串，然后保存配置，最后恢复数据输出

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

