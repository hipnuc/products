# CAN DBC 使用说明

本目录提供产品 CAN 报文解析所需的 DBC 文件，面向上位机调试、总线监控和数据解码使用。

## 文件说明

- `J1939.dbc`
  - 用于 J1939 协议数据解析
  - 适用于 INS（组合导航）和 IMU 的 J1939 数据显示

- `CANopen.dbc`
  - 用于 CANopen 协议数据解析
  - 适用于 IMU 的 CANopen TPDO 数据显示

## 快速上手

1. 打开你的 CAN 分析工具（如 CANdb++、CANalyzer、PCAN-View 等）。
2. 导入对应协议的 DBC 文件：
   - 使用 J1939 设备时导入 `J1939.dbc`
   - 使用 CANopen 设备时导入 `CANopen.dbc`
3. 连接设备并开始接收 CAN 报文，即可按信号名称查看物理量。

## 使用建议

- 默认节点 ID 按 `0x08` 建模。
- 若设备节点 ID 不是 `0x08`，请在工具中按实际 ID 做报文映射。
- 数据均按小端（Little-Endian）解析。
- 本目录 DBC 主要用于数据显示，不包含参数配置类控制报文定义。

## 协议选择建议

- INS（GNSS/组合导航）产品：使用 `J1939.dbc`
- IMU 产品：
  - 设备输出 J1939 时使用 `J1939.dbc`
  - 设备输出 CANopen 时使用 `CANopen.dbc`
