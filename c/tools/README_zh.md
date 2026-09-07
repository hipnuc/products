# HiPNUC Linux 工具

[English](README.md) | [中文](README_zh.md)

基于 `c/hipnuc` C 核心的两个命令行工具：

- `hihost` — 串口：查找设备、显示与记录数据流、发送 ASCII 命令、固件升级。
- `canhost` — SocketCAN：J1939 与 CANFD83 解码、寄存器读写、触发帧、CAN 固件升级。

两者仅支持 Linux（termios、SocketCAN）。`common/` 存放共用的辅助模块（日志、Intel HEX
加载、INI 读取）。

## 编译

```sh
cmake -S c/tools -B build/tools -DCMAKE_BUILD_TYPE=Release
cmake --build build/tools -j
build/tools/hihost/hihost --help
build/tools/canhost/canhost --help
```

每个工具也可单独编译：`cmake -S c/tools/hihost -B build/hihost`。需要 CMake 3.10
和 C99 编译器。

## hihost

```sh
hihost list                                # 列出串口
hihost probe --save                        # 查找端口/波特率并写入 ./hihost.ini
hihost read                                # 实时 JSON 显示
hihost -r raw.bin -j data.jsonl read       # 记录原始字节与 JSON 行
hihost write "LOG VERSION"                 # 发送一条 ASCII 命令
hihost write hihost/device_setup.ini       # 按文件逐行发送命令
hihost update firmware.hex                 # 串口固件升级
```

端口与波特率来自 `-p`/`-b`，否则依次查找 `$HIHOST_CONF`、`./hihost.ini`、
`~/.hihost.ini`（键 `port=`、`baud=`）。只有 `probe --save` 才会写文件。访问串口通常需要
加入 `dialout` 组。

`read` 每帧输出一个 JSON 对象（SI 单位，键名与 Python SDK 一致）。`update` 对正在运行的
设备和已处于 bootloader 的设备都可用。

## canhost

```sh
sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up

canhost device list
canhost device probe                       # J1939 地址声明扫描
canhost stream read                        # 每个解码帧一行 JSON
canhost stream record -o run.jsonl         # 同上，写入文件并带 rx_time_us
canhost trigger sync --count 1             # 触发 canhost.ini 中列出的 PGN
canhost config reg read 0x70
canhost config reg write 0x06 1
canhost action run version                 # reset / save 需要 --yes
canhost firmware update -f app.hex         # CAN 固件升级，遍历全部目标节点
canhost -n 8,9 stream read                 # 临时覆盖节点列表
```

配置文件依次查找 `$CANHOST_CONF`、`./canhost.ini`、`~/.canhost.ini`、`/etc/canhost.ini`；
`canhost/canhost.ini` 注释了全部键（接口、节点列表、主机源地址、CAN FD、`sync.<pgn>`
周期）。只解码 J1939 源地址在节点列表中的帧。寄存器应答按目标节点匹配，多台设备可共用
一条总线。
