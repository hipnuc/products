# canhost - HiPNUC CAN 主机工具

## 概述

`canhost` 是一个面向 HiPNUC IMU/ARHS 设备的 SocketCAN Linux 命令行工具。通过统一的 CLI 命令即可完成接口检测、J1939 探测、实时数据展示与记录、以及寄存器配置与动作执行，便于在 Linux/树莓派等平台快速评估产品表现。

## 功能

- **接口管理**：`list` 命令以表格展示所有物理 CAN 接口状态。
- **设备探测**：`probe` 基于 J1939 协议探测总线上的设备，列出地址与 NAME。
- **动作执行**：`action` 执行预定义的复杂操作序列（如复位、保存配置、校准等）。
- **寄存器操作**：`reg` 直接读写 J1939 配置寄存器（支持多节点广播）。
- **同步触发**：`sync` 根据配置周期性发送同步请求，触发数据上传。
- **实时数据**：`read` 自动识别 J1939/CANopen 协议，解析并输出 JSON 格式传感器数据。
- **数据记录**：`record` 高性能记录 NDJSON 数据到文件，支持缓冲与丢帧统计。

## 目录结构

```
examples/CAN/linux
├── CMakeLists.txt
├── README.md
├── canhost.ini         # 默认配置文件
├── can_interface.c/h   # SocketCAN 封装
├── commands.c/h        # 命令注册与分发
├── command_handlers.h  # 命令函数声明
├── commands/
│   ├── cmd_action.c    # action 命令（动作序列）
│   ├── cmd_list.c      # list 命令
│   ├── cmd_probe.c     # probe 命令
│   ├── cmd_read.c      # read 命令
│   ├── cmd_record.c    # record 命令
│   ├── cmd_reg.c       # reg 命令（寄存器读写）
│   └── cmd_sync.c      # sync 命令
├── config.c/h          # 配置文件解析
├── log.c/h             # 日志工具
├── j1939_reg_api.c/h   # J1939 寄存器协议实现
├── reg_seq.c/h         # 寄存器序列执行器
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

依赖：CMake 3.10+、GCC、pthread、libm、Linux SocketCAN 头文件。

## 快速开始

1.  **准备 CAN 接口**：
    
    ```bash
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 500000
    sudo ip link set can0 up
    ```
    
    CANFD 示例：
    ```bash
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 500000 dbitrate 4000000 fd on
    sudo ip link set can0 up
    ip -details link show can0
    ```
    
2.  **配置**：
    在源码目录或当前目录创建 `canhost.ini`（参考源码中的 `canhost.ini`）：
    ```ini
    interface=can0
    node_id=8
    ```

3.  **运行**：
    ```bash
    ./canhost list
    ./canhost probe
    ./canhost read
    ```

## 使用方式

### 配置文件

程序按以下顺序搜索配置文件：
`$CANHOST_CONF` → `examples/CAN/linux/canhost.ini` → `./canhost.ini` → `~/.canhost.ini` → `/etc/canhost.ini`

配置示例：
```ini
interface=can0      # 使用的接口
node_id=8,9         # 目标节点地址（支持逗号分隔多个）
sync.sa=0x55        # 主机源地址
sync.0xff34=100     # 配置 sync 命令：每 100ms 请求一次 PGN 0xFF34

# CANFD
canfd=1             # 使能CANFD
canfd.brs=1         # 使能BRS
canfd.data_bitrate=4000000
```
说明：
- CAN 接口速率由 `ip link` 配置，配置文件中的 CANFD/BRS/data_bitrate 仅用于程序内部记录与对齐设备配置。

### 命令详解

| 命令 | 说明 |
| ---- | ---- |
| `list` | 列出物理 CAN 接口及其状态（UP/DOWN）。 |
| `probe` | 扫描 J1939 总线设备，显示源地址 (SA) 和 NAME。 |
| `action` | 执行预定义动作序列。 |
| `reg` | 读写设备寄存器。 |
| `sync` | 发送同步请求触发数据。 |
| `read` | 实时显示解析后的 JSON 数据。 |
| `record` | 记录数据到文件。 |

#### 1. Action (动作执行)
执行预定义的寄存器操作流程。
```bash
./canhost action <name>
```
支持的动作（取决于固件版本与定义）：
- `reset`: 复位设备
- `save`: 保存当前配置
- `version`: 读取版本信息
- `mag_calib`: 执行磁场校准流程

#### 2. Reg (寄存器操作)
读写 J1939 配置寄存器（对应 Modbus 地址）。若配置了多个 `node_id`，会对所有目标节点执行操作。
```bash
./canhost reg read <addr>           # 读取地址 addr (hex/dec)
./canhost reg write <addr> <val>    # 写入值 val 到地址 addr
```
示例：
```bash
./canhost reg read 0x0001
./canhost reg write 0x0010 100
```

#### 3. Sync (同步触发)
根据配置文件中的 `sync.*` 条目发送请求。
```bash
./canhost sync                  # 按配置无限循环发送
./canhost sync -c 10            # 发送 10 次循环后退出
```

#### 4. Read (实时数据)
自动识别 HiPNUC J1939 和 CANopen 协议，解析标准帧、扩展帧与 CANFD0 帧。
```bash
./canhost read
```
输出为 JSON 行格式 (JSON Lines)，包含时间戳、节点 ID、帧类型及物理量数据。

#### 5. Record (数据记录)
高性能录制工具，使用大缓冲区减少 I/O 阻塞。
```bash
./canhost record -o data.jsonl
```
终端会实时显示接收速率 (RX FPS)、写入速率 (Write FPS) 及丢帧统计。

## 协议与解析

- **自动识别**：根据 CAN ID 特征区分 J1939（扩展帧）与 CANopen（标准帧）。
- **统一输出**：所有协议解析后统一转换为内部数据结构，并以 JSON 格式输出。
