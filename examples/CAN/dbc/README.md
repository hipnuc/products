# CAN DBC 文件说明

## 简介

本仓库包含 :

​	HI32 IMU/GNSS 组合导航系统的 CAN 数据库文件 (DBC)，HI32GNSS-J1939.dbc文件，用于定义HI32GNSS  CAN 总线通信的消息格式和信号映射。

​	IMU 惯性导航系统的 CAN 数据库文件 (DBC)，IMU-J1939.dbc、IMU-CANOpen.dbc文件，用于定义IMU  CAN 总线通信的消息格式和信号映射。

## 使用方法

### CANdb++ 使用

1. 打开 CANdb++ 软件
2. 选择 `File` → `Open` 导入 `HI32GNSS-J1939.dbc` 或者`IMU-J1939.dbc`或者`IMU-CANOpen.dbc`文件
3. 查看消息列表和信号定义
4. 配置 CAN 接口进行数据监控

## 注意事项

- HI32-J1939.dbc、IMU-J1939.dbc消息 ID 遵循 J1939 PGN 格式
- 数据采用小端格式编码
- 需要根据信号定义的缩放因子进行单位转换

