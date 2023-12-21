# HIPNUC Python lib #

本路径提供了一个Python示例代码，用于通过Python读取HIPNUC模块数据。

<u>**当前支持的Python版本为3.6及以上。**</u>

已测试环境:

Windows 10

ubuntu 16.04

## 使用方法 ##

- 安装依赖

  cd到当前目录，执行：

```
pip install -r requirements.txt
```

- import hipnuc并使用

  请参考demo.py中的代码：

  ```python
  from hipnuc_module import *
  import time
  
  if __name__ == '__main__':
  
      HI221GW = hipnuc_module('./config.json')
  
      while True:
          data = HI221GW.get_module_data()
  
          print(data)
  
          time.sleep(0.010)
  ```

  

## API说明

*class* `hipnuc_module`(*path_configjson=None*)

​	超核模组类，用于接收、处理超核模组信息。

​	参数:**path_configjson** (*str*) – json配置文件的路径.

​	`get_module_data`(*timeout=None*)

​			获取已接收到的模组数据.

​			参数

​			**timeout** – 可选参数。若为None(默认值),将会阻塞直至有有效值; 若timeout为正数，将会尝试等待有效数据并阻塞timeout秒,若阻塞时间到，但仍未有有效数据,将会抛出Empty异常.

​			返回：**data** – 返回模组数据，类型为字典

​			返回类型:dict

> 返回数据格式说明:
>
> **返回为字典。字典的key为数据类型，value为所有节点该数据的list。**
>
> 例如，返回数据中，第1个模组的加速度数据为data["acc"]\[0];返回数据中，第16个模组的四元数信息为data["quat"]\[15].

​	`get_module_data_size`()

​			获取已接收到的模组数据的数量. 注意:返回长度大于0,不保证`get_module_data`时不会被阻塞.

​			参数:**无**

​			返回:**size** – 返回模组数据，类型为字典

​			返回类型:int

​	`close`()

​			关闭指定的模组.

​			参数:**无** 

​			返回:**无**



## JSON 配置文件说明

在初始化hipnuc_module时，需要传入JSON配置文件的路径，并从配置文件中获取串口端口、波特率、数据类型等信息。配置文件如下所示：

```json
{
    "port": "COM11",
    "baudrate": 460800,
    "report_datatype": {
      "Expanding Information": false,
      "acc": true,
      "gyr": false,
      "quat": false
    }
}
```

配置含义如下：

**port**

串口端口，类型为**字符串**。在Windows下为`"COM*"`，例如`"COM11"`;Linux下,一般为`"/dev/tty*"`，例如`"/dev/ttyUSB0"`。请根据设备的实际情况进行配置。

**baud rate**

串口波特率，类型为**整型**。请根据模块实际参数进行设置。

**report_datatype**

汇报数据种类。模组将会上报多种数据信息，可通过本设置项，配置hipnuc_module实际解析的数据类型。

**IMUSOL**

​		数据包ID 0x91 ，包含加速度，角速度四元数，欧拉角等消息

​		若为`true`，hipnuc_module将会返回该数据，若为`false`，hipnuc_module不会返回该数据。

**GWSOL**

​		数据包ID 0x62 ，HI221Dongle 接收机消息，包含多个0x91数据包，返回所有无线节点的姿态数据

​		若为`true`，hipnuc_module将会返回该数据，若为`false`，hipnuc_module不会返回该数据。

**ACC**

​		数据包ID 0xA0，加速度信息。类型为**布尔型**。

​		若为`true`，hipnuc_module将会返回该数据，若为`false`，hipnuc_module不会返回该数据。

**GYR**

​		数据包ID 0xB0 ，角速度信息。类型为**布尔型**。

​		若为`true`，hipnuc_module将会返回该数据，若为`false`，hipnuc_module不会返回该数据。

**EUL**

​		数据包ID 0xD0，欧拉角整形格式。类型为**布尔型**。

​		若为`true`，hipnuc_module将会返回该数据，若为`false`，hipnuc_module不会返回该数据。

**QUAT**

​		数据包ID 0xD1 ，四元数信息。类型为**布尔型**。

​		若为`true`，hipnuc_module将会返回该数据，若为`false`，hipnuc_module不会返回该数据。

**ID**

​		数据包ID 0x90 ，用户ID。类型为**布尔型**。

​		若为`true`，hipnuc_module将会返回该数据，若为`false`，hipnuc_module不会返回该数据。

**MAG**

​		数据包ID 0xC0，磁场强度信息。类型为**布尔型**。

​		若为`true`，hipnuc_module将会返回该数据，若为`false`，hipnuc_module不会返回该数据。

## 附： ##

若在使用中遇到任何问题或建议，请与超核联系，谢谢。