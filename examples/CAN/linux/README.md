# canhost（Linux CAN 命令行工具）

`canhost` 是面向 HiPNUC 设备的 Linux 终端工具，支持设备探测、实时数据查看、数据落盘、同步触发和寄存器读写。

## 1. 编译

```bash
cd examples/CAN/linux
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

生成文件：`build/canhost`

## 2. 先配置 CAN 口

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

## 3. 配置文件

程序按以下顺序加载配置：

1. `$CANHOST_CONF`
2. 当前目录 `./canhost.ini`
3. `~/.canhost.ini`
4. `/etc/canhost.ini`

常用项：

```ini
interface=can0
node_id=8
sync.sa=0x55
sync.0xff34=100
sync.0xff37=100
```

## 4. 命令总览

```bash
canhost device list
canhost device probe
canhost stream read
canhost stream record -o imu.jsonl
canhost trigger sync --count 1
canhost trigger sync --pgn 0xff34 --count 10
canhost config reg read 0x70
canhost config reg write 0x06 1
canhost action run reset --yes
canhost help
canhost version
```

全局参数：

- `-n, --node ID[,ID...]`：临时覆盖配置文件中的目标节点列表。

## 5. 典型流程（现场调试）

1. 先确认接口：`canhost device list`
2. 探测在线设备：`canhost device probe`
3. 实时查看数据：`canhost stream read`
4. 长时间记录：`canhost stream record -o run.jsonl`
5. 需要触发上传时：`canhost trigger sync --count 1`

## 6. 输出说明

- `stream read` 与 `stream record` 默认输出 JSON（NDJSON）。
- CANopen 与 J1939 在以下字段单位已统一：
  - `acc_*`：`G`
  - `gyr_*`：`deg/s`
  - `roll/pitch/imu_yaw`：`deg`

## 7. 注意事项

- `action run` 中危险动作默认禁用，必须加 `--yes` 才执行。
- 若未配置 `node_id`，命令会直接报错并返回非 0。
- 本工具不依赖额外 Python 包，可直接编译使用。

