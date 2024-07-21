# Python 数据读取即模块配置例程 #

## 简介

本示例包提供了如何使用 Python 从 IMU 设备读取数据并进行处理的示例代码。支持 Windows、Ubuntu 和树莓派平台。

## 系统需求

- **Python**: 版本 3.6 及以上
- **依赖库**: pyserial 版本 3.4 及以上

## 测试平台

- Windows 11
- Ubuntu 20.04
- 树莓派 4B

## 安装与使用

### 1. 安装依赖

首先，确保你已经安装了 Python 3.6 及以上版本。然后，使用以下命令安装所需依赖：

```python
$ pip install -r requirements.txt
```

## 运行示例代码

运行 `main.py` 文件以开始从 IMU 设备读取数据。

在 Linux 系统上： 

```bash
$ sudo python main.py -p /dev/ttyUSB0 -b 115200
```

在 Windows 系统上

```sh
$ python main.py -p COM3 -b 115200
```

## 示例输出

成功运行后，程序将读取并打印 IMU 数据。以下是一个示例输出：

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

## 目录结构

.
├── main.py               # 主程序入口
├── README.md             # 项目说明文件
├── requirements.txt      # 依赖包列表
└── hipnuc_serial_parser.py # 二进制数据解析器

## 常见问题

### 如何更改串口配置？

你可以通过命令行参数 `-p` 和 `-b` 来指定串口端口和波特率。例如：

```bash
$ python main.py -p COM3 -b 115200
```

### 如果遇到权限问题怎么办？

在 Linux 系统上，访问串口设备可能需要超级用户权限。你可以使用 `sudo` 命令来运行脚本：

```bash
$ sudo python main.py -p /dev/ttyUSB0 -b 115200
```

