# HiPNUC Arduino 例程

HiPNUC IMU/INS 传感器 Arduino 库和示例，支持实时 JSON 数据输出。

## 功能特性

- **实时数据处理**: 连续解码 HiPNUC 协议数据包
- **JSON 输出格式**: 结构化数据输出，便于集成

## 支持硬件

### Arduino 开发板
- Arduino Mega 2560

## 硬件连接

```
HiPNUC 传感器    Arduino 开发板
--------------   -------------
VCC          ->  5V (ESP 使用 3.3V)
GND          ->  GND
TX           ->  (以Arduino Mega2560为例)RX1_P19
RX           ->  (以Arduino Mega2560为例)RT1_P18
```

## 快速开始

### 2. 配置硬件
按照上述连接图连接 HiPNUC 传感器到 Arduino 开发板。

### 3. 编译上传
1. 打开 Arduino IDE
2. 加载 `main.ino` 文件
3. 选择正确的开发板和端口
4. 编译并上传

### 4. 查看输出
打开串口监视器 (115200 波特率)，你将看到：

**启动信息**:
```
System Information:
MCU Frequency: 16 MHz
Baud Rate: 115200
Board: Arduino Mega2560
Firmware Version: 1.0
Output Format: JSON
------------------------
HiPNUC Decoder Ready
Starting in:
3...
2...
1...
Data acquisition started
========================
```

**JSON 数据输出**:
```json
{"fps":25.0,"time":4.2,"imu":{"acc":[0.123,0.045,9.812],"gyr":[0.12,-0.34,0.56],"euler":{"roll":1.23,"pitch":-0.43,"yaw":45.67}}}
{"fps":24.8,"time":4.4,"ins":{"lat":31.123456,"lon":121.654321,"sats":12}}
```

### 修改输出频率
```cpp
const int DISPLAY_RATE = 200;  // 200ms = 5Hz 输出
```

### 修改串口波特率
```cpp
Serial1.begin(115200);         // 传感器串口
```
