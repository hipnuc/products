# ROS2串口例程

本文档介绍如何在ROS2下来读取HI226/HI229/CH100/CH110的数据，并提供了c++语言例程代码，通过执行ROS2命令，运行相应的节点，就可以看到打印到终端上的信息。

* 测试环境：Ubuntu20.04   

* ROS版本：ROS2 Foxy

* 测试设备：HI226 HI229  CH100  CH110(USB)

## 1. 安装USB-UART驱动

Ubuntu 系统自带CP210x的驱动，默认不需要安装串口驱动。将调试版连接到电脑上时，会自动识别设备。识别成功后，会在dev目录下出现一个对应的设备:ttyUSBx

检查USB-UART设备是否被Ubantu识别：

1. 打开终端，输入`ls /dev`,先查看已经存在的串口设备。
2. 查看是否已经存在  ttyUSBx 这个设备文件，便于确认对应的端口号。x表示USB设备号，由于Ubuntu USB设备号为从零开始依次累加，所以多个设备每次开机后设备号是不固定的，需要确定设备的设备号。
4. 接下来插入USB线，连接调试板，然后再次执行`ls /dev`。 dev目录下多了一个设备, 如图：

<img src="https://raw.githubusercontent.com/hipnuc/products/master/examples/ROS2/img/2.png">

**ttyUSB0** 文件就是调试版在ubuntu系统中生成的设备(后面的数字是不固定的，有可能为 ttyUSB1  或 ttyUSB2)

5. 打开USB设备的可执行权限：

```shell
   $ sudo chmod 777 /dev/ttyUSB0
```

## 2. 编译serial_imu_ws工作空间

1. 打开终端进入/examples/ROS2/serial_imu_ws 目录
2. 执行`colcon build`命令，编译成功后出现如下信息。

```shell
linux@ubuntu20:~/serial_imu_ws$ colcon build
Starting >>> serial_imu
Finished <<< serial_imu [0.24s]                  

Summary: 1 package finished [0.35s]
linux@ubuntu20:~/serial_imu_ws$ 
```

## 3. 修改串口波特率和设备号

1. 在Ubuntu环境中，支持的波特率为115200, 460800, 921600。本例程使用的默认波特率是115200，默认打开的串口名称是/dev/ttyUSB0。	

2. 如果您需要更高的输出频率，请修改serial_port.cpp文件中的宏定义，改为其他波特率。	

```c
#define IMU_SERIAL  ("/dev/ttyUSB0")
#define BAUD        (B115200)
```

注意修改后需要回到serial_imu_ws目录下，重新执行`colcon build`命令

## 4. 显示数据
本例程提供了一种查看数据方式：

​	1、输出ROS 定义的sensor_msgs::Imu。

###  4.1：输出ROS标准 Imu.msg

1. 在windows系统下进行配置模块，使能四元数输出。
2. 使用Window下 CHCenter上位机进行配置：先把模块连接到PC机上。然后使用CHCenter工具进行 连接对应的com口，点击 __工具__  --->  __配置模块__，在弹出的新窗口中，点击__ATCMD__，然后在输入框中输入AT指令：`AT+SETPTL=0x91`，点击发送，接收区最后显示 __ok__ ，说明配置成功，断电重启模块。执行`ros2 launch serial_imu imu_spec_msg.launch.py`命令。执行成功后，就可以看到ROS定义的IMU话题消息：

```c
[listener-2] ---
[listener-2] header:
[listener-2] 	stamp:
[listener-2] 	  secs:1639099575
[listener-2] 	  nanosecs:538349240
[listener-2] 	frame_id:base_link
[listener-2] orientation:
[listener-2] 	x: -0.095125280320644379
[listener-2] 	y: -0.483648955821990967
[listener-2] 	z: 0.053129896521568298
[listener-2] 	w: 0.868453860282897949
[listener-2] orientation_covariance: [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[listener-2] angular_velocity: 
[listener-2] 	x: -0.000815955184543841
[listener-2] 	y: -0.001057390143056437
[listener-2] 	z: 0.001062464062371403
[listener-2] angular_velocity_covariance: [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[listener-2] linear_acceleration:
[listener-2] 	x: 8.110355603694916482
[listener-2] 	y: -2.125157430768013000
[listener-2] 	z: 5.013053989410400924
[listener-2] linear_acceleration_covariance: [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[listener-2] ---
```

​	3、另开一个终端窗口，执行`ros2 topic hz /Imu_data`，可以查看话题发布的频率。

```shell
linux@ubuntu20:~$ ros2 topic hz /Imu_data 
average rate: 100.032
	min: 0.008s max: 0.012s std dev: 0.00058s window: 102
average rate: 100.014
	min: 0.008s max: 0.012s std dev: 0.00054s window: 202
average rate: 100.019
	min: 0.007s max: 0.013s std dev: 0.00064s window: 303
^C
linux@ubuntu20:~$ 

```



## 5. FAQ

1、有时候主板上需要插好多的usb设备，为了方便开发，通常会编写一个usb端口约束文件。如果是不同型号的usb设备，可以通过设备的id号来区分。如果是同型号的设备，他们的id号都是一样的，这个时候就需要更多的细分信息来区分不同的usb设备。接下来就操作一下如何区分同型号的usb设备。

```shell
linux@ubuntu:~$ lsusb
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 012: ID 10c4:ea60 Cygnal Integrated Products, Inc. CP210x UART Bridge / myAVR mySmartUSB light
Bus 002 Device 011: ID 10c4:ea60 Cygnal Integrated Products, Inc. CP210x UART Bridge / myAVR mySmartUSB light
Bus 002 Device 010: ID 10c4:ea60 Cygnal Integrated Products, Inc. CP210x UART Bridge / myAVR mySmartUSB light
Bus 002 Device 008: ID 0e0f:0008 VMware, Inc. 
Bus 002 Device 003: ID 0e0f:0002 VMware, Inc. Virtual USB Hub
Bus 002 Device 002: ID 0e0f:0003 VMware, Inc. Virtual Mouse
Bus 002 Device 001: ID 1d6b:0001 Linux Foundation 1.1 root hub
linux@ubuntu:~$ 
```

​	观察上面的内容，发现有三个usb设备的id号完全一样，使用简单的id号区分行不通了，需要更多的设备信息。

```shell
linux@ubuntu:~$ ls /dev
agpgart          loop3               shm       tty32  tty63      ttyS7
autofs           loop4               snapshot  tty33  tty7       ttyS8
block            loop5               snd       tty34  tty8       ttyS9
bsg              loop6               sr0       tty35  tty9       ttyUSB0
btrfs-control    loop7               stderr    tty36  ttyprintk  ttyUSB1
bus              loop-control        stdin     tty37  ttyS0      ttyUSB2
......(未全部放出)
```

到这一步，dev文件中产生三个usb设备文件，分别是：ttyUSB0，ttyUSB1，ttyUSB2。

现在先看ttyUSB0的详细信息：

```shell
linux@ubuntu:~$ udevadm info --attribute-walk --name=/dev/ttyUSB0
#通过这个命令可以查看指定端口的详细信息
......
    ATTRS{devpath}=="2.2"
    ATTRS{idProduct}=="ea60"
    ATTRS{idVendor}=="10c4"
    ATTRS{ltm_capable}=="no"
    ATTRS{manufacturer}=="Silicon Labs"
    ATTRS{maxchild}=="0"
    ATTRS{product}=="CP2104 USB to UART Bridge Controller"
    ATTRS{quirks}=="0x0"
    ATTRS{removable}=="unknown"
    ATTRS{serial}=="01E34546"
......(信息太多了，就不全部放出来了，大家可以自己去看看详细的信息,这里只放出本次需要关心的信息)
```

然后是ttyUSB1的详细信息：

```shell
linux@ubuntu:~$ udevadm info --attribute-walk --name=/dev/ttyUSB1
#通过这个命令可以查看指定端口的详细信息
......
    ATTRS{devpath}=="2.3"
    ATTRS{idProduct}=="ea60"
    ATTRS{idVendor}=="10c4"
    ATTRS{ltm_capable}=="no"
    ATTRS{manufacturer}=="Silicon Labs"
    ATTRS{maxchild}=="0"
    ATTRS{product}=="CP2102N USB to UART Bridge Controller"
    ATTRS{quirks}=="0x0"
    ATTRS{removable}=="unknown"
    ATTRS{serial}=="9c1d818b48aeeb119d082897637728c5"
......(信息太多了，就不全部放出来了，大家可以自己去看看详细的信息,这里只放出本次需要关心的信息)
```

最后是ttyUSB2的详细信息：

```shell
linux@ubuntu:~$ udevadm info --attribute-walk --name=/dev/ttyUSB2
#通过这个命令可以查看指定端口的详细信息
......
    ATTRS{devnum}=="27"
    ATTRS{devpath}=="2.4"
    ATTRS{idProduct}=="ea60"
    ATTRS{idVendor}=="10c4"
    ATTRS{ltm_capable}=="no"
    ATTRS{manufacturer}=="Silicon Labs"
    ATTRS{maxchild}=="0"
    ATTRS{product}=="CP2104 USB to UART Bridge Controller"
    ATTRS{quirks}=="0x0"
    ATTRS{removable}=="unknown"
    ATTRS{serial}=="02228956"
......(信息太多了，就不全部放出来了，大家可以自己去看看详细的信息,这里只放出本次需要关心的信息)
```

通过上边的三个串口设备的信息，发现ATTRS{serial}=="xxxx"这一项，看起来特别随意。实际上这个是硬件的id号，也是硬件的唯一id号，通过这个号，给它起一个别名，这样一来，只要这个硬件id号被识别到，dev下就会出现自定义的端口名称设备文件，实现永久绑定端口号。

```shell
linux@ubuntu:~$ cd /etc/udev/rule.d/
linux@ubuntu:/etc/udev/rules.d$ ls
70-snap.core.rules  70-ttyusb.rules  99-vmware-scsi-udev.rules
#这一步是看看都有哪些约束文件，避免文件名重复
linux@ubuntu：~$ sudo vi defined_serial.rules
#这一步自定义一个串口约束文件名称，后缀为'.rules'
```

然后在这个文件中输入如下内容：

<img src="https://raw.githubusercontent.com/hipnuc/products/master/examples/ROS2/img/6.png">

格式如下：

```shell
KERNEL=="ttyUSB*", ATTRS{serial}=="xxx", ATTRS{idVendor}=="xxx", ATTRS{idProduct}=="xxx", MODE:="0777（端口的权限）",SYMLINK+="(自定义名称)"
```

把对应的信息填对，最后保存并退出文件，执行：

```shell
linux@ubuntu:~$ service udev reload
root privileges required
linux@ubuntu:~$ service udev restart
linux@ubuntu:~$ ls /dev
agpgart          loop1               sg1       tty32  tty7       ttyS9
autofs           loop2               shm       tty33  tty8       ttyUSB0
block            loop3               snapshot  tty34  tty9       ttyUSB1
BLUETOOCH        loop4               snd       tty35  ttyprintk  ttyUSB2
....
CH110            mcelog              tty0      tty40  ttyS13     vcs1
....
HI226            rfkill              tty22     tty54  ttyS27     vfio
....
```

现在可以看到，自定义的usb端口名称已经出来了，在操作的时候，直接操作对应的设备文件就好了，不用去理会端口的编号是多少了。