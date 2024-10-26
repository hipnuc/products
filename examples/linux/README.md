# 	Linux例程


## 概览
本工程在提供Linux上读取，配置HiPNUC产品的功能及示例，并提供linux命令行工具hihost的源代码及用法。hihost是一个用于读取和控制HiPNUC产品的命令行工具。它支持多种操作，包括列出可用串口、读取IMU数据、发送命令到设备，以及处理示例数据。

### 支持的硬件

 所有超核IMU产品

### 测试环境
- Ubuntu 20.04 
- 树莓派4B

### 文件说明

| 文件           | 说明                                     |
| -------------- | ---------------------------------------- |
| main           | 主程序入口，处理命令行参数和调用相应命令 |
| commands       | 实现各种命令的功能                       |
| serial_port    | Linux C串口驱动封装                      |
| log            | log组件                                  |
| fw_downloader  | 固件下载相关                             |
| CMakeLists.txt | CMake构建配置文件                        |

另外本工程依赖解码库hipnuc_dec.c 和 nmea_decode.c，位于 ../lib下:
| 文件            | 说明                                                         |
| --------------- | ------------------------------------------------------------ |
| hipnuc_dec.c    | 位于../lib下, HiPNUC二进制协议解析器                         |
| nmea_decode.c   | 位于../lib下, NMEA 消息解析器，支持GGA,RMC以及HiPNUC自定义NMEA 协议SXT 等 |
| example _data.c | 使用示例                                                     |


## 构建说明

本项目使用CMake构建，创建构建目录并进入: example/linux目录:

```
mkdir build
cd build
cmake ..
make
```

构建完成后生成可执行文件 `hihost` 

## 使用说明

### 基本用法

hihost [全局选项] <命令> [命令选项]

### 全局选项

- `-p, --port PORT`：指定串口设备（例如：/dev/ttyUSB0）
- `-b, --baud RATE`：指定波特率（默认：115200）
- `-h, --help`：显示帮助信息
- `-v, --version`：显示版本信息

### 可用命令

1. `list`：列出所有可用的串口
2. `probe`: 自动探测设备
3. `read`：进入读取模式，显示IMU数据
4. `write <COMMAND>`：向设备发送命令
5. `write <CONFIG_FILE>`：读取配置文件并根据配置内容向设备批量写入命令
6. `example`：处理示例静态数据
7. `update <HEX_FILE>`： 固件升级

### 使用示例

#### 列出可用串口
```sh
sudo ./hihost list
```

#### 自动探测设备

```sh
sudo ./hihost probe
```

一旦自动探测成功，后面使用其他命令后可省略 -p, -b 参数。

#### 读取IMU数据

```sh
sudo ./hihost -p /dev/ttyUSB0 -b 115200 read
```

这将持续显示IMU数据，如下所示：

```
acc(G):            0.020   -0.017    1.001
gyr(deg/s):        0.097    0.105   -0.194
mag(uT):          -8.808   20.392  -48.683
eul(deg):         -1.122   -1.102   -0.269
quat;              1.000   -0.010   -0.010   -0.002
presure(pa):    104514.461
timestamp(ms):   1286394
```

按 CTRL+C 可以退出程序, 如果需要将解码数据保存到文件可直接重定向到文件：

```sh
sudo ./hihost -p /dev/ttyUSB0 -b 115200 read > output.txt
```

#### 发送单个命令到设备
```sh
sudo ./hihost -p /dev/ttyUSB0 -b 115200 write "LOG VERSION"
```

这将发送 `LOG VERSION\r\n` 命令并显示返回数据：
```
Received 80 bytes:

PNAME=HI14R3
BUILD=Oct 15 2024
UUID=043995698D6E1708
APP_VER=156
BL_VER=109
OK
```
注意：使用 write 命令时，不需要在命令末尾添加 \r\n，程序会自动处理。

#### 使用配置文件批量发送命令

 hihost 支持从配置文件执行多个命令。配置文件是一个包含多个命令的文本文件，每行一个命令。使用方法如下：

```
sudo ./hihost -p /dev/ttyUSB0 -b 115200 write ../device_setup.ini
```

其中device_setup.ini 是配置文本文件，每一个命令占一行。配置文件[示例](device_setup.ini)

#### 处理静态示例数据
```sh
sudo ./hihost example
```
这将处理内置的示例数据并显示结果。

#### 固件升级

其中firmware.hex为升级固件，请联系我司获取，升级成功后会自动重启模块

```
sudo ./hihost -p /dev/ttyUSB0 -b 115200 update firwamre.hex
```

### 使用解码库驱动

如果您想加入数据解析工程到您的工程中,可以直接参考 ../lib下 hipnuc_dec.c 和 nmea_dec.c,  使用方法可以参考example_data.c

### 注意事项

* 在Linux系统中，访问串口设备通常需要root权限，因此需要使用 sudo 运行程序。
* 确保您有正确的串口设备名称和波特率设置。
