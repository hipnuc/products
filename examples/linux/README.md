# 	Linux例程

在Linux环境下接收超核IMU 二进制数据帧并显示

* 测试环境： Ubuntu 20.04 / 树莓派4B
* 支持硬件: 	所有超核IMU产品

## 文件说明

| 文件                          | 位置 | 说明                             |
| ----------------------------- | ---- | -------------------------------- |
| serial_port.c / serial_port.h | .    | linux C串口驱动封装              |
| ch_serial.c / ch_serial.h     | ../  | 公共驱动文件，解析超核二进制协议 |
| main                          | .    | example示例文件                  |

## 使用

1. 查找串口设备,  确定Linux可以找到你的tty串口，命令行切换到本目录下执行make，生成可执行文件main
```shell
 $ ls /dev/ttyUSB*
```

2. 编译example, 切到本目录下执行make. 编译为可执行文件main

```
$ make
gcc -I../lib -c main.c -o main.o
gcc -I../lib -c serial_port.c -o serial_port.o
gcc -I../lib -c ../lib/ch_serial.c -o ../lib/ch_serial.o
gcc -I../lib main.o serial_port.o ../lib/ch_serial.o -o main
Cleaning up...
```

3. 运行main
```
$ sudo ./main ttyUSB0 115200
isatty success!
* ttyUSB0 successfully open with 115200.
* Starting serial data reader. Press CTRL+C to exit.
* Select mode:
  R - Read device data
  C - Send command to device
* Enter your choice: 

```

4. 根据提示输入

   * R+回车:  读取并显示IMU数据
   * C+回车:  向模块发送一条ASCII命令

执行R后会打印IMU数据：
```
acc(G):            0.020   -0.017    1.001
gyr(deg/s):        0.097    0.105   -0.194
mag(uT):          -8.808   20.392  -48.683
eul(deg):         -1.122   -1.102   -0.269
quat;              1.000   -0.010   -0.010   -0.002
presure(pa):    104514.461
timestamp(ms):   1286394
```

执行C后会根据提示输入ASCII命令, 如AT+INFO, 而后会返回接收到的串口的字节数和数据。
```
* Enter your choice: C
Enter command(example: AT+INFO) or Press CTRL+C to exit:
AT+INFO
Received 150 bytes: 

HI14 1.3.8 build Dec 27 2023
2010 - 2023 Copyright by HiPNUC
MODE:           6 AIXS
UUID:           043995698D6E1708
ODR:            100Hz
OK
```
