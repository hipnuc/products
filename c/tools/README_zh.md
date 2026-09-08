[English](README.md) | [中文](README_zh.md)

# 固件升级与 CAN 工具

串口发现、配置和录制使用 [Python SDK](../../python/README_zh.md)。
这里提供 Windows/Linux 串口升级工具，以及 Linux SocketCAN 工具。

## 构建

安装 C 编译器和 CMake 后，在本目录执行：

```sh
cmake -S . -B build
cmake --build build --config Release
```

Linux 可执行文件为 `build/serial_update/hipnuc-update`、`build/canhost/canhost`；
Windows 使用 Visual Studio 时，升级程序为
`build\serial_update\Release\hipnuc-update.exe`。

## 串口升级

使用与设备型号匹配的 Intel HEX 固件，指定当前连接速度和端口：

```powershell
.\build\serial_update\Release\hipnuc-update.exe firmware.hex -p COM3 -b 115200
```

Linux：

```sh
./build/serial_update/hipnuc-update firmware.hex -p /dev/ttyUSB0 -b 115200
```

升级失败后停止，不自动重启。传输及重启请求得到确认，不代表已验证新程序成功启动。

## CAN（Linux）

先按设备实际速率启动 CAN 接口，例如：

```sh
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

在本目录运行：

```sh
./build/canhost/canhost list
./build/canhost/canhost scan -i can0 --duration 2
./build/canhost/canhost read -i can0 -n 8 --duration 10
./build/canhost/canhost read -i can0 -n 8 --record samples.jsonl
```

`scan` 观察有效测量报文，无法发现静默设备。`read` 将 JSONL 输出到 stdout
或 `--record` 文件，诊断进入 stderr。省略 `-n` 时接收全部源地址。
Ctrl-C、`--count`、`--duration` 均在处理完当前批次后停止；
只有显式指定 `--overwrite` 才覆盖已有记录文件。

寄存器地址、数值含义和适用型号以产品手册为准。
直接使用原始数值，支持十进制和 `0x` 十六进制：

```text
./build/canhost/canhost reg read ADDRESS -i can0 -n 8
./build/canhost/canhost reg write ADDRESS VALUE -i can0 -n 8
./build/canhost/canhost sync PGN -i can0 -n 8 --interval 0.01 --count 10
./build/canhost/canhost update firmware.hex -i can0 -n 8
```

仅使用产品支持的地址和触发 PGN。保存、重启需显式写对应寄存器；
普通写入不会自动保存。寄存器请求使用主机地址 `0x55`，与设备回复协议一致。
CAN 升级支持设备地址 1–127，只有升级流程使用 CANopen SDO；
原始二进制固件使用 `--bin`。

在最终子命令后加 `--help` 查看参数。工具不读取 INI，也不修改接口波特率。
退出码：0 成功，1 运行失败，2 参数错误，130 Ctrl-C。
