# CAN DBC 使用说明

本目录提供 HiPNUC 产品的 CAN 解码文件，面向终端用户在上位机工具中直接导入使用。

## 文件列表

- `J1939.dbc`
  - 适用：INS（GNSS/组合导航）与 IMU 的 J1939 数据解析
- `CANopen.dbc`
  - 适用：IMU 的 CANopen TPDO 数据解析

## 如何选择

1. 设备输出 J1939：使用 `J1939.dbc`
2. 设备输出 CANopen：使用 `CANopen.dbc`

## 使用步骤

1. 在 CAN 分析工具中导入对应 DBC（如 CANdb++、CANalyzer、PCAN-View 等）。
2. 连接设备并开始接收报文。
3. 按信号名称查看物理量。

## 关键说明

- 默认节点 ID 按 `0x08` 建模。
- 信号按 little-endian（小端）解析。
- DBC 仅用于数据显示与解析，不包含控制/配置类报文定义。

