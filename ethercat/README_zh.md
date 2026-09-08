# EtherCAT 例程（HI15）

[English](README.md) | [中文](README_zh.md)

在 Linux 上使用 IgH EtherCAT Master **1.6** 读取 HI15 的加速度、角速度、四元数和温度。
修改 `main.c` 顶部参数后编译运行。默认主站 0、别名 0、位置 0，启用 DC，
总线周期 1 kHz，显示频率 20 Hz。Ctrl-C 停止并释放主站。

## 准备与运行

1. 安装 IgH 1.6（包括用户态开发库）、C 编译器和 CMake 3.16 或更新版本。
   按照 [IgH 安装说明](https://gitlab.com/etherlab.org/ethercat/-/blob/stable-1.6/INSTALL.md)
   配置专用以太网接口并启动主站服务。
2. 连接 HI15 并供电。运行 `ethercat slaves -v`，确认从站位置、厂商 ID
   `0x00131415` 和产品代码 `0x00009253`。关闭其他使用此主站的应用程序。
3. 在本目录执行：

```sh
cmake -S . -B build && cmake --build build
sudo ./build/userexample
```

`sudo` 用于访问主站设备；如果系统已经授予当前账户访问权限，可以省略。
IgH 安装在自定义前缀时，在 CMake 命令后添加 `-DCMAKE_PREFIX_PATH=/path/to/igh`。

普通 Linux 内核也可用于初次读取。1 kHz DC 同步对调度延迟敏感，本例程不保证实时性。
持续同步运行请参考 [IgH 实时性指导](https://etherlab.org/en_GB/getting-started)，
无需为了开始使用例程而先自行编译内核。

## 数据与集成

`hi15.c` 提供固定 PDO 映射及主站资源管理，`main.c` 展示接收、处理、读取和发送循环。
将它们和 `hi15.h` 复制到自己的 IgH 工程即可；同一主站保持一个资源所有者和一个周期接收循环。

- 加速度：**m/s²**；角速度：**rad/s**；温度：**°C**。
- 四元数：**WXYZ**，使用设备当前配置的参考坐标系；例程不修改设备配置或转换坐标。
- `system_time`：设备运行时间，单位**毫秒**，32 位回绕。
- 仅显示运行状态正常且完整交换的 PDO。设备时间可能重复，一次总线交换不一定对应新样本。
- RxPDO `0x7000:01` 为保留字段，写入零。

[HI15 ESI 文件](https://download.hipnuc.com/esi/hi15_esi.zip) 提供主站配置所需的设备标识及 PDO 描述。
本 C 例程直接使用固定映射，运行时不读取 XML。

未枚举到从站时，检查供电、接线和主站网卡。程序显示 `No valid PDO` 时，检查输出的 AL 状态、
从站位置及标识是否匹配，并查看主站日志。有效交换恢复后会继续显示测量，不把先前读数当成当前数据。
