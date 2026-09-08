[English](README.md) | [中文](README_zh.md)

# C 与 C++

解码 HiPNUC 串口二进制、NMEA、J1939/CANFD83 数据，也可以直接在 Windows/Linux
应用中读取设备。C++ 使用同一套 C API。

## 读取设备

修改 [read.c](examples/read.c) 或 [read.cpp](examples/read.cpp) 顶部的
`PORT`、`BAUDRATE`，在本目录构建：

```sh
cmake -S examples -B build/examples
cmake --build build/examples --config Release
```

Linux 运行 `./build/examples/read_c` 或 `./build/examples/read_cpp`；Windows 使用 Visual Studio
构建时运行 `.\build\examples\Release\read_c.exe` 或 `.\build\examples\Release\read_cpp.exe`。
Ctrl-C 停止并关闭连接。Ubuntu 需要串口权限：执行
`sudo usermod -aG dialout "$USER"`，然后注销并重新登录。

## 集成到自己的工程

- **MCU 或已有收发代码：** 按[核心库说明](hipnuc/README_zh.md)复制所需文件。
  C99 核心不依赖堆内存或操作系统。
- **CMake：** 使用 `add_subdirectory(path/to/c/hipnuc hipnuc)`；二进制/NMEA
  链接 `hipnuc_core`，CAN 链接 `hipnuc_j1939`。JSON 格式化按需链接。
- **桌面串口：** 改为引入 `path/to/c/serial` 并链接 `hipnuc_serial`。
  每个连接使用一个零初始化的 `hipnuc_serial_t`；返回值和资源归属见
  [公共头文件](serial/hipnuc_serial.h)。

```c
hipnuc_sample_t sample;
int result = hipnuc_serial_read_sample(&device, &sample, 200);
if (result == 1 && (sample.valid & HIPNUC_VALID_ACC)) {
    /* sample.acc[] is specific force in m/s^2, with gravity not removed. */
}
```

每个样本对应一份新收到的报文。仅在有效位存在时读取字段；缺失不代表零。
保留设备坐标配置，INS 与原始 GNSS 数据分别存放。

设备发现、配置和串口录制使用 [Python SDK](../python/README_zh.md)；
固件升级及 SocketCAN 使用[专用工具](tools/README_zh.md)。
