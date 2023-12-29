# python示例 #

* Python需求：
  * Python > 3.6
  * pyserial >=3.4
* 测试平台
  * Windows 11
  * Ubuntu 20.04
  * 树莓派4B

## 使用方法 ##

1. 安装依赖(如果没有的话)

```
$ pip install -r requirements.txt
```

或

```
$ sudo pip3 install pyserial
```

2. 运行main.py, 用法: `python main.py <port> <baudrate>` 

* 例(Linux): `$ sudo python main.py /dev/ttyUSB0 115200`
* 例(Window) `$ python main.py COM3 115200`

成功运行后会读取IMU数据并打印:

```
Temperature  :      0 °C
Pressure     :   500.000 Pa
Acceleration : X=    0.184, Y=   -0.840, Z=    0.413 m/s²
Gyroscope    : X=   -0.094, Y=   -0.030, Z=   -0.012 °/s
Magnetometer : X=    0.006, Y=    0.010, Z=    0.008 μT
Euler Angles : Roll=  -24.205, Pitch=  -61.690, Yaw=   -0.014 °
Quaternion   : W=    0.839, X=   -0.501, Y=   -0.180, Z=    0.107
Timestamp    : 830522 ms
```
