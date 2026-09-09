[English](README.md) | [中文](README_zh.md)

# C 与 C++

小型 C99 解码库，支持 HiPNUC 二进制、NMEA、J1939/CANFD83 数据。
核心不依赖堆内存、操作系统或全局可变状态。Windows/Linux 应用还可使用
同步串口接口；C++ 直接调用同一套 C API。

## MCU 或已有接收代码

从 [hipnuc/](hipnuc) 复制所需文件。每行相互独立，已列全依赖；
将对应 `.c` 文件及头文件目录加入工程。

| 输入或输出 | 文件 |
| --- | --- |
| 串口二进制 HI91/HI81/HI83 | `hipnuc_dec.c/.h`、`hipnuc_sample.c/.h` |
| NMEA GGA/RMC | `nmea_dec.c/.h`、`hipnuc_sample.c/.h` |
| CAN J1939/CANFD83 | `hipnuc_j1939.c/.h`、`hipnuc_can_frame.h`、`hipnuc_sample.c/.h` |
| 可选 JSON 格式化 | `hipnuc_json.c/.h`、`hipnuc_sample.c/.h` |

把串口收到的字节送入解码器：

```c
#include "hipnuc_dec.h"

static hipnuc_raw_t decoder; /* zero-initialized; one per input stream */

void on_byte(uint8_t byte)
{
    hipnuc_sample_t sample;
    if (hipnuc_input(&decoder, byte) > 0) {
        hipnuc_sample_from_raw(&decoder, &sample);
        if (sample.valid & HIPNUC_VALID_ACC) {
            /* Use sample.acc[0..2], in m/s^2, in your application. */
        }
    }
}
```

`hipnuc_input()` 返回 `1` 表示收到完整支持帧，`0` 表示继续等待，`-1` 表示无效帧。
错误后继续送入后续字节即可。中断中只接收并缓存字节，主循环中调用解码器；
[STM32 例程](../stm32/README_zh.md) 提供接线说明及完整 Keil 工程。

NMEA 使用 `nmea_input()` 和 `hipnuc_sample_from_nmea()`。
CAN 将驱动收到的帧填入 `hipnuc_can_frame_t`，调用 `hipnuc_j1939_parse()`；
返回正数表示新样本。多设备总线须检查 `node_id`。

## Windows 或 Linux 读取设备

需要 CMake 3.16 及以上版本和 C 编译器：Windows 使用带 C++ 构建工具的
Visual Studio / Build Tools，Linux 使用 GCC 或 Clang。

修改所选例程顶部的参数：

| 示例 | 用途 |
| --- | --- |
| [read.c](examples/read.c) | C 串口读取 |
| [read.cpp](examples/read.cpp) | C++ 调用同一套 C 串口 API |
| [read_can.c](examples/read_can.c) | Linux SocketCAN，支持 Classic CAN 和 CAN FD |

从仓库根目录构建：

```sh
cmake -S c -B build/c
cmake --build build/c --config Release
```

Linux 运行 `./build/c/examples/read_c`、`./build/c/examples/read_cpp`
或 `./build/c/examples/read_can`。
Windows 使用 Visual Studio 时运行 `.\build\c\examples\Release\read_c.exe`
或 `.\build\c\examples\Release\read_cpp.exe`。
检测到 C++ 编译器时才构建 C++ 例程。
Ctrl-C 停止并关闭连接。这些示例用于集成；逐条打印不适合高速数据录制。

Ubuntu 串口权限通常通过 `sudo usermod -aG dialout "$USER"` 设置，然后注销并重新登录。
设置正确的端口和波特率；设备输出较慢时增加 `TIMEOUT_MS`。
SocketCAN 先按设备波特率启动适配器，例如 Classic CAN 500 kbit/s：

```sh
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

CAN FD 还需要支持它的适配器及匹配的数据段波特率。

## CMake 集成

```cmake
add_subdirectory(path/to/c hipnuc)
target_link_libraries(my_app PRIVATE hipnuc_core)
```

二进制/NMEA 选 `hipnuc_core`，CAN 选 `hipnuc_j1939`，可选 JSON 格式化选
`hipnuc_json`；它们共用独立的 `hipnuc_sample` target。作为子项目时只构建
调用方链接的库，默认不构建例程；C 应用无需 C++ 编译器。
链接目标会自动传递头文件目录和 C99 要求。

Windows/Linux 串口应用在 `add_subdirectory()` 前将 `HIPNUC_BUILD_SERIAL`
设为 `ON`，然后链接 `hipnuc_serial`。从清零的 `hipnuc_serial_t` 开始，
按指定端口和波特率打开、读取，最后关闭。`hipnuc_serial_read_sample()` 返回
`1` 表示新样本、`0` 表示超时、`-1` 表示失败。资源归属及超时约定见
[hipnuc_serial.h](serial/hipnuc_serial.h)。已有工程也可直接包含 `c/hipnuc` 或 `c/serial`。

## 数据约定

只有对应 `HIPNUC_VALID_*` 位存在时才读取字段。这些位表示**有此字段**，
不代表定位有效或姿态收敛。每个样本只代表当前报文，CAN 不混入旧 PGN 数据。
Roll/pitch、yaw、heading 相互独立；INS 与原始 GNSS 位置分别保留。
HI83 倾角字段中的 yaw 使用独立字段及有效位，与欧拉角 yaw 分别保留。

NMEA 原始 GGA/RMC 结构体改用整数 `second_ms`，替代原来的 `second`。
需要浮点秒时使用 `second_ms / 1000.0`；SI 样本的时间字段不变。

样本使用 SI 单位：加速度 m/s²（未去除重力）、角速度 rad/s、角度 rad、磁场 T、
气压 Pa；经纬度用度，温度用 °C。保留设备坐标配置，不隐式转换。
单位、状态及质量字段见 [hipnuc_sample.h](hipnuc/hipnuc_sample.h)。
收敛判断使用转换后的字段：设备 `ATT_CONV`/`WB_CONV` 位为 1 表示警告。

核心要求 8 位字节和 IEEE 754 浮点数；
位置字段要求 8 字节 `double`。不支持的二进制布局和 HI83 bitmap 字段会被拒绝。
仅可选 JSON 格式化模块使用 `stdio`。

桌面配置和固件升级使用[官方下载](https://download.hipnuc.com)中的 CHCenter。
自动发现、命令、录制、CAN 操作及无界面固件升级使用
[Python SDK](../python/README_zh.md)。
