# canhost - HiPNUC CAN 主机工具

## 概述

`canhost` 是一个面向 HiPNUC IMU/ARHS 设备的 SocketCAN Linux 命令行工具。通过统一的 CLI 命令即可完成接口检测、J1939 探测、实时数据展示，便于在 Linux/树莓派等平台快速评估产品表现。

## 功能

- **接口一览**：`list` 命令以表格展示所有物理 CAN 接口（忽略 `vcan/vxcan`）
- **实时数据**：`read` 输出解析后的 JSON 行，自动缓存多种帧类型，按 `Ctrl+C` 退出。


## 目录结构

```
examples/CAN/linux
├── CMakeLists.txt
├── README.md
├── can_interface.c/h       # SocketCAN 枚举 & socket 工具
├── commands.c/h            # 命令分发
├── command_handlers.h
├── commands/
│   ├── cmd_list.c          # list 命令
│   ├── cmd_probe.c         # probe 命令
│   ├── cmd_read.c          # read 命令
├── config.c/h
├── log.c/h
└── utils.c/h
```

## 构建方法

```bash
cd examples/CAN/linux
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# 生成的可执行文件为 build/canhost
```

依赖：CMake 3.10+、GCC、libm、Linux SocketCAN 头文件。

## 快速开始（适合新用户）

- 准备 CAN 接口：确保物理连接；若为 USB 适配器，加载对应驱动后应出现 `can0/slcan0` 等接口。
- 手动拉起接口（示例 500 kbit/s）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

- 配置文件写入接口名：在源码同级或当前目录提供 `canhost.ini`，内容为：

```
interface=can0
```

- 运行并观察：

```bash
./canhost list      # 查看接口状态
./canhost read      # 终端显示（约10 Hz，各类型最新帧）
./canhost read -o imu.json  # 同时完整记录到文件
```

## 使用方式

### 配置文件

程序不再通过命令行指定接口，而是从简单的初始化配置文件读取：

- 搜索顺序：`$CANHOST_CONF` → `examples/CAN/linux/canhost.ini` → `./canhost.ini` → `~/.canhost.ini` → `/etc/canhost.ini`
- 配置格式（ini 风格，键值对）：

```
interface=can0
```

#### 示例配置文件

- 仓库建议将配置文件放置在源码同级目录：`examples/CAN/linux/canhost.ini`

说明：如果未提供配置文件，程序使用默认值 `interface=can0`。

命令行仅保留：

| 选项 | 说明 |
| ---- | ---- |
| `-h, --help` | 显示英文帮助 |
| `-v, --version` | 显示版本号 |

### 命令

| 命令 | 作用 |
| ---- | ---- |
| `list` | 展示接口状态 |
| `probe` | 设备探测，输出地址及 NAME |
| `read` | 实时显示 HiPNUC 传感器数据（JSON 行） |

示例：

```bash
./canhost list
./canhost probe
./canhost read
./canhost read -o imu.json
```

## 协议与解析

- **设备探测**：`probe` 发送 PGN 0xEA00（REQUEST）请求 ADDRESS_CLAIMED，并列出所有响应节点。（仅针对 J1939 设备）
- **HiPNUC-CAN 数据**：`read` 按帧类型自动路由：扩展帧由 `hipnuc_j1939_parser` 解析，标准帧由 `canopen_parser` 解析；统一 JSON 输出由 `hipnuc_can_common` 提供。节点 ID 来自 CAN 帧 ID 解析。


## 注意事项

1. 访问 SocketCAN 通常需要 root 权限，若遇到 `Operation not permitted` 请使用 `sudo`.
2. `list` 仅列出真实物理接口，`vcan`、`vxcan` 默认忽略；`slcan*` 会被视为物理口。
3. 若未提供配置文件，则默认 `interface=can0`。

## 故障排查

| 现象 | 排查建议 |
| ---- | -------- |
| `list` 无接口 | 检查硬件连接并加载驱动(以PeakCAN为例)（如 `sudo modprobe peak_usb`） |
| `read` 无数据 | 检查接口是否 UP、波特率匹配、设备是否确实输出帧 |
