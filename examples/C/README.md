# Linux例程

## 概览
本工程在 Linux 上提供读取、配置 HiPNUC 产品的功能与示例，并提供命令行工具 `hihost` 的源代码及用法。`hihost` 用于读取和控制 HiPNUC 产品，支持列出串口、自动探测设备、读取 IMU/GNSS 数据、发送命令以及固件升级等。

### 支持的硬件
- 所有超核 IMU 产品

### 测试环境
- Ubuntu 20.04
- 树莓派 4B

## 文件说明
| 文件/目录        | 说明                                   |
| ---------------- | -------------------------------------- |
| `main.c`         | 主程序入口，处理命令行参数并调用命令   |
| `commands.c`     | 命令注册与分发（模块化后保留统一入口） |
| `commands/`      | 各命令的独立模块实现目录               |
| `cmd_utils.{h,c}`| 通用辅助函数（如 `safe_sleep`）         |
| `serial_port.{h,c}` | 串口驱动封装                         |
| `log.{h,c}`      | 简易日志输出组件                       |
| `fw_downloader/` | 固件下载相关实现                       |
| `CMakeLists.txt` | CMake 构建配置文件                     |

另外本工程依赖解码库，位于 `../drivers` 下：
| 文件             | 说明                                                         |
| ---------------- | ------------------------------------------------------------ |
| `hipnuc_dec.c`   | HiPNUC 二进制协议解析器                                      |
| `nmea_dec.c`     | NMEA 消息解析器，支持 GGA、RMC 以及 HiPNUC 自定义 NMEA `SXT` |
| `example_data.c` | 示例数据用法参考                                             |

## 项目结构（模块化命令架构）
- `commands.c`：仅维护命令注册表与分发函数 `execute_command(...)`。
- `commands/cmd_list.c`：列出可用串口。
- `commands/cmd_probe.c`：自动探测设备端口与波特率，并保存到 `device_setup.ini`。
- `commands/cmd_probe.c`：自动探测设备端口与波特率，并保存到 `hihost.ini`。
- `commands/cmd_read.c`：读取 HipNuc/NMEA 数据，支持原始与 JSON 录制、FPS 统计与实时显示。
- `commands/cmd_write.c`：发送单条或批量（从文件） AT 命令并处理响应。
- `commands/cmd_update.c`：固件升级（HEX→BIN 转换、KBOOT 交互、写入与进度显示）。
- `commands/cmd_example.c`：处理示例静态数据。

以上模块由统一头 `command_handlers.h` 声明，`main.c` 与 `commands.h` 的外部接口保持不变，确保向后兼容。

## 构建说明
### Linux
```sh
mkdir -p build
cd build
cmake ..
make
```
生成可执行文件 `hihost`。

## 配置文件（hihost.ini）

程序使用源码目录下的配置文件 `examples/C/hihost.ini` 来存储串口与波特率，键值为：

```
port=/dev/ttyUSB0
baud=115200
```

- 固定路径：仅识别 `examples/C/hihost.ini`
- 自动更新：`probe` 成功后会写回探测到的 `port` 与 `baud`

### Windows（可选）
请安装以下工具之一：
- Visual Studio Build Tools（含 MSVC、CMake）
- MinGW-w64 + `cmake` + `ninja`（推荐）

示例（MinGW + Ninja）：
```powershell
mkdir build
cd build
cmake .. -G "Ninja"
ninja
```
如使用 NMake，请确保 `nmake` 在 PATH 中，否则 CMake 会报错。

## 使用说明
### 基本用法
```
hihost [全局选项] <命令> [命令参数]
```

### 全局选项
- `-p, --port PORT`：指定串口设备（例如：`/dev/ttyUSB0`）
- `-b, --baud RATE`：指定波特率
- `-r, --record-raw FILE`：记录原始串口数据到二进制文件
- `-j, --record-json FILE`：记录解析后的 JSON 数据到文本文件
- `-h, --help`：显示帮助信息
- `-v, --version`：显示版本信息

### 可用命令
1. `list`：列出所有可用的串口
2. `probe`：自动探测设备端口与波特率（结果写入 `device_setup.ini`）
3. `read`：进入读取模式，显示 IMU/GNSS 数据
4. `write <COMMAND>`：向设备发送单条命令（无需手动添加 `\r\n`）
5. `write <CONFIG_FILE>`：从配置文件批量发送命令（每行一条）
6. `example`：处理示例静态数据
7. `update <HEX_FILE>`：固件升级

### 使用示例
#### 列出可用串口
```sh
sudo ./hihost list
```

#### 自动探测设备
```sh
sudo ./hihost probe
```
探测成功后，会把结果写入 `examples/C/hihost.ini`，后续命令可省略 `-p`、`-b` 参数。若修改了设备波特率，请重新执行 `probe`。

#### 读取 IMU 数据
```sh
sudo ./hihost -p /dev/ttyUSB0 -b 115200 read
```
或在已配置 `hihost.ini` 的情况下：
```sh
sudo ./hihost read
```
示例输出：
```
acc(G):            0.020   -0.017    1.001
gyr(deg/s):        0.097    0.105   -0.194
mag(uT):          -8.808   20.392  -48.683
eul(deg):         -1.122   -1.102   -0.269
quat;              1.000   -0.010   -0.010   -0.002
presure(pa):    104514.461
timestamp(ms):   1286394
```
按 `CTRL+C` 退出。

#### 数据记录功能
1) 原始数据记录（`-r`）
```sh
sudo ./hihost -p /dev/ttyUSB0 -b 115200 -r imu_raw.bin read
```
2) JSON 数据记录（`-j`）
```sh
sudo ./hihost -p /dev/ttyUSB0 -b 115200 -j imu_data.json read
```
3) 同时记录两种格式
```sh
sudo ./hihost -p /dev/ttyUSB0 -b 115200 -r raw.bin -j data.json read
```
数据格式：
- 原始文件（`.bin`）：完整串口数据流，用于回放与深度分析
- JSON 文件（`.json`）：每行一个 JSON 对象，便于二次处理

#### 发送单个命令到设备
```sh
sudo ./hihost -p /dev/ttyUSB0 -b 115200 write "LOG VERSION"
```
返回示例：
```
PNAME=HI14R3
BUILD=Oct 15 2024
UUID=043995698D6E1708
APP_VER=156
BL_VER=109
OK
```

#### 使用配置文件批量发送命令
```sh
sudo ./hihost -p /dev/ttyUSB0 -b 115200 write ../device_setup.ini
```
配置文件每行一条命令，示例见 `device_setup.ini`。

#### 固件升级
```sh
sudo ./hihost -p /dev/ttyUSB0 -b 115200 update firmware.hex
```
固件请联系我司获取；升级成功后模块会自动重启。

## 使用解码库驱动
若需在你的工程中集成数据解析，可直接参考 `drivers` 下的 `hipnuc_dec.c` 与 `nmea_dec.c`，使用方法可参考 `example_data.c`。

## 开发指南（新增命令）
1. 在 `examples/C/commands/` 新增 `cmd_<name>.c`，实现 `int cmd_<name>(GlobalOptions *opts, int argc, char **argv)`。
2. 在 `examples/C/command_handlers.h` 中声明该函数。
3. 在 `examples/C/commands.c` 的命令注册表中添加：`{"<name>", cmd_<name>, "<desc>"}`。
4. 在 `examples/C/CMakeLists.txt` 的 `SOURCES` 列表中加入新源文件。

## 注意事项
- 在 Linux 系统中访问串口通常需要 root 权限，建议使用 `sudo` 运行。
- 请确认串口设备名称与波特率设置正确。
- Windows 构建需正确安装并配置工具链；若使用 NMake，请确保 `nmake` 可用。
