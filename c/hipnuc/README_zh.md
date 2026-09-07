# HiPNUC C 核心

[English](README.md) | [中文](README_zh.md)

面向 HiPNUC 设备（固件 1.6.9 及以上）的可移植 C99 解码器与固件升级客户端。
无动态内存、不依赖操作系统、无全局状态，解码器不使用 `stdio`。可用 GCC、Clang、
MinGW、MSVC、ARM Compiler 5/6 编译。字段布局、单位和状态位都写在头文件注释里。

## 按需复制文件

| 用途 | 文件 |
| --- | --- |
| 串口二进制流（HI91/HI81/HI83） | `hipnuc_dec.c/.h` |
| + SI 单位和有效标志 | `hipnuc_sample.c/.h`、`nmea_dec.h` |
| + NMEA `$GPGGA` / `$GPRMC` | `nmea_dec.c` |
| + JSON 输出（用到 `stdio`） | `hipnuc_json.c/.h` |
| CAN：J1939 与 CANFD83 | `hipnuc_j1939.c/.h`、`hipnuc_can_frame.h`、`hipnuc_sample.c/.h`、`hipnuc_dec.h`、`nmea_dec.h` |
| 串口固件升级 | `hipnuc_kboot.c/.h`、`hipnuc_dec.c/.h` |
| CAN 固件升级 | `hipnuc_can_update.c/.h`、`hipnuc_can_frame.h` |

或用 CMake：

```cmake
add_subdirectory(path/to/c/hipnuc hipnuc)
target_link_libraries(my_app PRIVATE hipnuc_core)   # hipnuc_json、hipnuc_j1939、hipnuc_update
```

## 串口

```c
static hipnuc_raw_t raw;                 /* 每个串口一个，零初始化 */

void on_byte(uint8_t byte)               /* 你的串口接收路径 */
{
    hipnuc_sample_t s;
    if (hipnuc_input(&raw, byte) > 0 && hipnuc_sample_from_raw(&raw, &s)) {
        if (s.valid & HIPNUC_VALID_EULER) use(s.roll, s.pitch, s.yaw);   /* rad */
        if (!s.attitude_converged) keep_still();
    }
}
```

`hipnuc_input()` 收齐一帧返回 1，需要更多字节返回 0，帧损坏返回 -1，请判断 `> 0`。
`hipnuc_input_buffer()` 整块喂入并在第一帧后停止。`hipnuc_sample_t` 是带逐字段有效位的
SI 视图，`hipnuc_raw_t` 里的原始报文保持线上单位。NMEA 用 `nmea_input()` 和
`hipnuc_sample_from_nmea()`，用法相同。

容易踩的坑：

- 状态位 `WB_CONV` / `ATT_CONV` **置位表示未收敛**。请用 `s.attitude_converged`、
  `s.gyro_bias_converged`，不要直接读原始位。
- HI91 加速度线上单位是 G，SDK 按 1 G = 9.8 m/s²（固件常数，不是 9.80665）换算。
- 带内部位 25–29 的 HI83 帧会被整帧拒收，不做部分解码。

## CAN

```c
hipnuc_can_frame_t frame;   /* 用驱动收到的 id、is_extended、len、data 填充 */
hipnuc_sample_t part, merged = {0};
if (hipnuc_j1939_parse(&frame, &part, NULL) > 0 && part.node_id == 8)
    hipnuc_j1939_merge(&merged, &part);
```

每个 PGN 只填它携带的字段；同一源地址的各部分由你自行合并。CANFD83（PGN 0xFF5B）
按位图头解码。

## 固件升级

直接使用现成工具：`c/tools` 下的 `hihost update`（串口）或 `canhost firmware update`（CAN）。
需要在自己的程序里集成升级功能时，见 `hipnuc_kboot.h` 与 `hipnuc_can_update.h` 的接口说明。

## 约束

8 位字节、IEEE 754 浮点、8 字节 `double`（4 字节 `double` 的平台会拒收带位置的 HI83 帧）。
与主机字节序无关。头文件可被 C++ 包含。一个上下文拥有一个解码器即可，在中断里喂字节没问题。

测试在 `c/tests`（CMake + ctest），包含"只复制文件"的工程和一个 C++ 调用方。
