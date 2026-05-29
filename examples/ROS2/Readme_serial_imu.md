# ROS2 IMU 串口例程

本文档介绍如何在ROS2下来读取超核电子IMU的数据，并提供了c++语言例程代码，通过执行ROS2命令，运行相应的节点，就可以看到打印到终端上的信息。

* 测试环境：Ubuntu20.04   

* ROS版本：ROS2 Foxy

* 测试设备：超核电子IMU系列产品

## 安装USB-UART驱动

Ubuntu 系统自带CP210x的驱动，默认不需要安装串口驱动。将调试版连接到电脑上时，会自动识别设备。识别成功后，会在dev目录下出现一个对应的设备:ttyUSBx

检查USB-UART设备是否被Ubantu识别：

1. 打开终端，输入`ls /dev`,先查看已经存在的串口设备。
2. 查看是否已经存在  ttyUSBx 这个设备文件，便于确认对应的端口号。
4. 接下来插入USB线，连接调试板，然后再次执行`ls /dev`。 dev目录下多了一个设备`ttyUSB0`：

```shell
linux@ubuntu:~$ ls /dev
.....
hpet             net           tty11     tty4   ttyS0      ttyUSB0    vhost-vsock
hugepages        null          tty12     tty40  ttyS1      udmabuf  vmci
......
```

4.打开USB设备的可执行权限：

```shell
   $ sudo chmod 777 /dev/ttyUSB0
```

## 编译hipnuc_ws工作空间

1. 打开终端进入/examples/ROS2/hipnuc_ws 目录
2. 执行`colcon build`命令，编译成功后出现如下信息。

```shell
linux@ubuntu20:~/examples/ROS2/hipnuc_ws$ colcon build
Starting >>> hipnuc_lib_package
Finished <<< hipnuc_lib_package [1.05s]                     
Starting >>> hipnuc_gnss
Starting >>> hipnuc_imu
Finished <<< hipnuc_gnss [1.15s]                                                               
Starting >>> hipnuc_imu_can
Finished <<< hipnuc_imu_can [0.77s]                                                               
Finished <<< hipnuc_imu [10.2s]                      

Summary: 4 packages finished [11.5s]
linux@ubuntu20:~/examples/ROS2/hipnuc_ws$ 
```

##  修改串口波特率和设备号

1. 在Ubuntu环境中，支持的波特率为115200, 460800, 921600。本例程使用的默认波特率是115200，默认打开的串口名称是/dev/ttyUSB0，默认发布ros::Imu.msg topic。	

2. 如果您需要更高的输出频率，请先用win上位机修改模块的波特率，然后再修改`config/hipnuc_config.yaml`文件中的配置参数。	

```c
IMU_publisher:
    ros__parameters:
        serial_port: "/dev/ttyUSB0"
        baud_rate: 115200
        frame_id: "imu_link"
        imu_switch: TRUE
        imu_topic: "/IMU_data"
        euler_switch: FALSE
        euler_topic: "/euler_data"
        magnetic_switch: FALSE
        magnetic_topic: "/magnetic_data"
        temperature_switch: FALSE
        temperature_topic: "/temp_data"
        pressure_switch: FALSE
        pressure_topic: "/pressure_data"
```

注意修改后需要回到hipnuc_ws目录下，重新执行`colcon build`命令

## 显示数据
​	查看数据方式：

​	1、输出ROS 定义的sensor_msgs::Imu。

​	2、输出ROS 定义的sensor_msgs::msg::MagneticField

​	3、输出ROS定义的geometry_msgs::msg::Vector3Stamped（Euler)

​	4、输出ROS定义的sensor_msgs::msg::Temperature

​	5、输出ROS定义的sensor_msgs::msg::FluidPressure

### 输出ROS标准 Imu.msg

1. 打开终端，执行：

```shell
linux@ubuntu20:~$ ros2 launch hipnuc_imu imu_spec_msg.launch.py
```

​	2.如果执行失败，提示找不到相应的launch文件，则需要配置环境，在当前终端执行：

```shell
linux@ubuntu:~$source <hipnuc_ws_dir>/install/setup.bash
```

​	3.执行成功后，就可以看到所有的信息：

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

​	4、另开一个终端窗口，执行`ros2 topic hz /Imu_data`，可以查看话题发布的频率。

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

### 输出ROS标准的sensor_msgs::msg::MagneticField

​	1、修改配置文件hipnuc_config.yaml --->  magnetic_switch: TRUE

```shell
IMU_publisher:
    ros__parameters:
        serial_port: "/dev/ttyUSB0"
        baud_rate: 115200
        frame_id: "imu_link"
        imu_switch: TRUE
        imu_topic: "/IMU_data"
        euler_switch: FALSE
        euler_topic: "/euler_data"
        magnetic_switch: TRUE
        magnetic_topic: "/magnetic_data"
        temperature_switch: FALSE
        temperature_topic: "/temp_data"
        pressure_switch: FALSE
        pressure_topic: "/pressure_data"
```

​	2、保存配置文件

​	3、启动节点

```shell
linux@ubuntu20:~$ ros2 launch hipnuc_imu imu_spec_msg.launch.py
```

​	4、另开一个终端窗口，执行`ros2 topic hz /magnetic_data`，可以查看话题发布的频率。

```shell
linux@ubuntu20:~$ ros2 topic hz /magnetic_data`
average rate: 100.028
	min: 0.001s max: 0.018s std dev: 0.00151s window: 101
average rate: 100.055
	min: 0.001s max: 0.018s std dev: 0.00124s window: 202
average rate: 99.963
	min: 0.001s max: 0.018s std dev: 0.00149s window: 302
average rate: 99.999
	min: 0.001s max: 0.018s std dev: 0.00146s window: 403
average rate: 99.980
	min: 0.001s max: 0.018s std dev: 0.00151s window: 503
^C
linux@ubuntu20:~$
```

### 输出ROS标准的geometry_msgs::msg::Vector3Stamped（Euler)

​	1、修改配置文件hipnuc_config.yaml --->  euler_switch: TRUE

```shell
IMU_publisher:
    ros__parameters:
        serial_port: "/dev/ttyUSB0"
        baud_rate: 115200
        frame_id: "imu_link"
        imu_switch: TRUE
        imu_topic: "/IMU_data"
        euler_switch: TRUE
        euler_topic: "/euler_data"
        magnetic_switch: FALSE
        magnetic_topic: "/magnetic_data"
        temperature_switch: FALSE
        temperature_topic: "/temp_data"
        pressure_switch: FALSE
        pressure_topic: "/pressure_data"
```

​	2、保存配置文件

​	3、启动节点

```shell
linux@ubuntu20:~$ ros2 launch hipnuc_imu imu_spec_msg.launch.py
```

​	4、另开一个终端窗口，执行`ros2 topic hz /euler_data`，可以查看话题发布的频率。

```
linux@ubuntu20:~$ ros2 topic hz /euler_data
average rate: 100.008
	min: 0.001s max: 0.020s std dev: 0.00143s window: 1307
average rate: 100.007
	min: 0.001s max: 0.020s std dev: 0.00141s window: 1408
average rate: 100.005
	min: 0.001s max: 0.020s std dev: 0.00141s window: 1508
average rate: 99.992
	min: 0.001s max: 0.020s std dev: 0.00143s window: 1608
average rate: 100.004
```

### 输出ROS标准的sensor_msgs::msg::Temperature

​		1、修改配置文件hipnuc_config.yaml --->  temperature_switch: TRUE

```shell
IMU_publisher:
    ros__parameters:
        serial_port: "/dev/ttyUSB0"
        baud_rate: 115200
        frame_id: "imu_link"
        imu_switch: TRUE
        imu_topic: "/IMU_data"
        euler_switch: FALSE
        euler_topic: "/euler_data"
        magnetic_switch: FALSE
        magnetic_topic: "/magnetic_data"
        temperature_switch: TRUE
        temperature_topic: "/temp_data"
        pressure_switch: FALSE
        pressure_topic: "/pressure_data"
```

​	2、保存配置文件

​	3、启动节点

```shell
linux@ubuntu20:~$ ros2 launch hipnuc_imu imu_spec_msg.launch.py
```

​	4、另开一个终端窗口，执行`ros2 topic hz /temp_data`，可以查看话题发布的频率。

```
linux@ubuntu20:~$ ros2 topic hz /temp_data
average rate: 100.011
	min: 0.001s max: 0.019s std dev: 0.00159s window: 64
average rate: 100.015
	min: 0.001s max: 0.020s std dev: 0.00160s window: 164
average rate: 100.011
	min: 0.001s max: 0.020s std dev: 0.00152s window: 264
average rate: 100.006
	min: 0.001s max: 0.020s std dev: 0.00149s window: 365
average rate: 100.003
	min: 0.001s max: 0.020s std dev: 0.00151s window: 465
average rate: 100.011

```



### 输出ROS标准的sensor_msgs::msg::FluidPressure

​		1、修改配置文件hipnuc_config.yaml --->  pressure_switch: TRUE

```shell
IMU_publisher:
    ros__parameters:
        serial_port: "/dev/ttyUSB0"
        baud_rate: 115200
        frame_id: "imu_link"
        imu_switch: TRUE
        imu_topic: "/IMU_data"
        euler_switch: FALSE
        euler_topic: "/euler_data"
        magnetic_switch: FALSE
        magnetic_topic: "/magnetic_data"
        temperature_switch: FALSE
        temperature_topic: "/temp_data"
        pressure_switch: TRUE
        pressure_topic: "/pressure_data"
```

​	2、保存配置文件

​	3、启动节点

```shell
linux@ubuntu20:~$ ros2 launch hipnuc_imu imu_spec_msg.launch.py
```

​	4、另开一个终端窗口，执行`ros2 topic hz /pressure_data`，可以查看话题发布的频率。

```
linux@ubuntu20:~$ ros2 topic hz /pressure_data
average rate: 100.002
	min: 0.001s max: 0.020s std dev: 0.00137s window: 17
average rate: 99.999
	min: 0.001s max: 0.020s std dev: 0.00138s window: 118
average rate: 99.979
	min: 0.001s max: 0.020s std dev: 0.00139s window: 219
average rate: 100.002
	min: 0.001s max: 0.020s std dev: 0.00139s window: 319

```

