# HiPNUC C 核心

[English](README.md) | [中文](README_zh.md)

面向当前 HiPNUC 产品（固件 1.6.9 及以上）的小型 C99 解码器。
无堆分配、不依赖操作系统、无全局可变状态，解码器不使用 `stdio`。
桌面串口连接和 C/C++ 示例从 [C SDK](../README_zh.md) 开始。

## 按需复制文件

每行均列出所需的全部文件；二进制、NMEA 和 CAN 解码互不依赖。

| 用途 | 文件 |
| --- | --- |
| 串口二进制 HI91/HI81/HI83，含 SI 样本 | `hipnuc_dec.c/.h`、`hipnuc_sample.c/.h` |
| NMEA GGA/RMC，含 SI 样本 | `nmea_dec.c/.h`、`hipnuc_sample.c/.h` |
| CAN J1939/CANFD83，含 SI 样本 | `hipnuc_j1939.c/.h`、`hipnuc_can_frame.h`、`hipnuc_sample.c/.h` |
| 单独使用 JSON 格式化（用到 `stdio`） | `hipnuc_json.c/.h`、`hipnuc_sample.c/.h` |
| 串口固件升级 | `hipnuc_kboot.c/.h`、`hipnuc_dec.c/.h`、`hipnuc_sample.c/.h` |
| CAN 固件升级 | `hipnuc_can_update.c/.h`、`hipnuc_can_frame.h` |

也可以链接 CMake 目标：

```cmake
add_subdirectory(path/to/c/hipnuc hipnuc)
target_link_libraries(my_app PRIVATE hipnuc_core)
# Other targets: hipnuc_json, hipnuc_j1939, hipnuc_update
```

## 接收字节

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

`hipnuc_input()` 收齐支持的完整帧返回 `1`，等待字节返回 `0`，无效帧返回 `-1`；
错误后继续输入后续字节。每个二进制外层帧只能包含一个支持的子包。
NMEA 对应使用 `nmea_input()` 和 `hipnuc_sample_from_nmea()`。
MCU 应在中断中缓存数据、在主循环中解码；[STM32 示例](../../stm32/README_zh.md) 演示了这个安排。

CAN 驱动每收到一帧，填入 `hipnuc_can_frame_t` 并调用 `hipnuc_j1939_parse()`；
返回正值表示新样本。多设备总线需要检查 `node_id`。每个样本只包含当前帧的字段，
不会自动拼接不同 PGN。

## 使用测量字段

`hipnuc_sample_t.valid` 是 64 位**字段可用标志**。缺失字段中存放的零不代表测量值。
roll/pitch 与 yaw、水平位置与高度、升沉位移与频率分别有独立标志。
INS 位置和原始 GNSS 位置也使用独立字段。判断定位是否有效时，还应检查 GNSS 质量
或 RMC 的状态和模式。

单位和坐标约定见 `hipnuc_sample.h`；SDK 保留设备的坐标配置。
HI91 加速度按固件常数 1 G = 9.8 m/s² 还原；HI81 输出 heading，不能当作 Euler yaw。
`WB_CONV`、`ATT_CONV` 置位表示**未收敛**；存在 `HIPNUC_VALID_STATUS` 时，
可直接使用转换后的收敛字段。

解码器要求 8 位字节和 IEEE 754 浮点。二进制 64 位位置字段要求 8 字节 `double`；
不支持的布局和 HI83 位图会被拒收。头文件可直接被 C++ 包含。

固件升级命令见[工具说明](../tools/README_zh.md)。集成升级功能时，实现
`hipnuc_kboot.h` 或 `hipnuc_can_update.h` 中的回调。CAN 固件升级是唯一使用 CANopen SDO 的功能。
