# canhost - HiPNUC CAN 主机工具

## 概述

`canhost` 是一个面向 HiPNUC IMU/ARHS 设备的 SocketCAN 命令行工具。通过统一的 CLI 命令即可完成接口检测、J1939 探测、实时数据展示以及帧速率统计，便于在 Linux/树莓派等平台快速评估产品表现。

## 功能

- **接口一览**：`list` 命令以表格展示所有物理 CAN 接口（忽略 `vcan/vxcan`）
- **实时数据**：`read` 以 ANSI 表格实时刷新 IMU 数据，自动缓存多种帧类型，按 `Ctrl+C` 即可退出。
- **帧率统计**：`stats` 统计每个 CAN ID 的速率与 DLC，并显示总帧率与活跃 ID 数量。

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
│   └── cmd_stats.c         # stats 命令
├── global_options.h
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

## 使用方式

### 全局选项

| 选项 | 说明 |
| ---- | ---- |
| `-i, --interface IFACE` | 指定 SocketCAN 接口（如 `can0`、`slcan0`） |
| `-n, --node-id ID` | 目标节点 ID（0-255，默认 8） |
| `-h, --help` | 显示英文帮助 |
| `-v, --version` | 显示版本号 |

### 常用命令

| 命令 | 作用 |
| ---- | ---- |
| `list` | 展示接口状态 |
| `probe` | 设备探测，输出地址及 NAME |
| `read` | 实时显示 HiPNUC 传感器数据 |
| `stats` | 监控各 CAN ID 的帧率与 DLC |

示例：

```bash
./canhost list
./canhost -i can0 probe
./canhost -i can0 -n 8 read
./canhost -i can0 stats
```

## CAN 接口快速配置

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

`list` 命令会读取 `/sys/class/net/<iface>/can_bittiming/*`，若接口 down 或驱动未导出该节点则显示 `N/A`。

## 协议与解析

- **设备探测**：`probe` 发送 PGN 0xEA00（REQUEST）请求 ADDRESS_CLAIMED，并列出所有响应节点。(probe只针对J1939协议设备)
- **HiPNUC-CAN 数据**：`read` 依赖 `hipnuc_can_parser` 解析 ACCEL/GYRO/MAG/EULER/QUAT/PRESSURE/INCLI/TIME 等帧，10 Hz 刷新终端。
- **CAN 帧统计**：`stats` 不解析载荷，仅按 ID 聚合频率，适合观察总线负载与丢包情况。

## 注意事项

1. 访问 SocketCAN 通常需要 root 权限，若遇到 `Operation not permitted` 请使用 `sudo`.
2. `list` 仅列出真实物理接口，`vcan`、`vxcan` 默认忽略；`slcan*` 会被视为物理口。

## 故障排查

| 现象 | 排查建议 |
| ---- | -------- |
| `list` 无接口 | 检查硬件连接并加载驱动(以PeakCAN为例)（如 `sudo modprobe peak_usb`） |
| `read` 无数据 | 检查节点 ID、波特率与是否确实有 HiPNUC 帧输出 |
