# 配置例程

## 使用方法

### 	修改配置文件

​	本例程目录下，有一个名为 __hipnuc_config.yaml__ 的文件,把配置命令写入到这个文件中，例如：

```shell
commands:
    - LOG IMU91 ONTIME 0.002
    - SERIALCONFIG 921600
    - SAVECONFIG
```

​	commands：顶格书写，不要修改这个标签

   - LOG IMU91 ONTIME 0.002    接下就是配置命令，根据用户自己的需求，进行修改这部分的内容，修改完之后，保存文件。

### 	python配置例程

#### 使用要求

* Python需求
  * Python > 3.6
  * pyserial >=3.4
* 测试平台
  * Ubuntu20.04
  * 树莓派4B  & JETSON NANO

#### 安装依赖

```shell
linux@ubuntu20:~$ sudo pip3 install pyserial
Requirement already satisfied: pyserial in /usr/local/lib/python3.8/dist-packages (3.5)
```

#### 执行文件

```shell
python3 config.py --port /dev/ttyUSB0 --config ./hipnuc_config.yaml
```

示例：

```shell
linux@ubuntu20:~/pyconfig$ python3 config.py --port /dev/ttyUSB0 --config ./hipnuc_config.yaml
Retrying ...4 attempts left
Retrying ...3 attempts left
Retrying ...2 attempts left
Retrying ...1 attempts left
Retrying ...0 attempts left
Retrying ...4 attempts left
send command:AT+EOUT=0

received:OK

send command:AT+EOUT=0

received:OK

send command:AT+INFO

received:
HI14R2-URT-000 1.5.1 build Apr  2 2024
MODE:           6 AIXS
UUID:           06C944246D12170F
ODR:            100Hz
OK

send command:LOG IMU91 ONTIME 0.002

received:OK

send command:SERIALCONFIG 921600

received:OK

send command:SAVECONFIG

received:OK

Reconnect the IMU and check the new configuration!
Reconnect the IMU and check the new configuration!
Retrying ...4 attempts left
Retrying ...3 attempts left
Retrying ...2 attempts left
Retrying ...1 attempts left
Retrying ...0 attempts left
Retrying ...4 attempts left
Retrying ...3 attempts left
Retrying ...2 attempts left
Retrying ...1 attempts left
Retrying ...0 attempts left
Retrying ...4 attempts left
Retrying ...3 attempts left
Retrying ...2 attempts left
Retrying ...1 attempts left
Retrying ...0 attempts left
Retrying ...4 attempts left
Retrying ...3 attempts left
Retrying ...2 attempts left
Retrying ...1 attempts left
Retrying ...0 attempts left
Retrying ...4 attempts left
Retrying ...3 attempts left
Retrying ...2 attempts left
Retrying ...1 attempts left
Retrying ...0 attempts left
Retrying ...4 attempts left
Retrying ...3 attempts left
send command:AT+EOUT=0

received:OK

new baud rate: 921600
send command:AT+INFO

received:
HI14R2-URT-000 1.5.1 build Apr  2 2024
MODE:           6 AIXS
UUID:           06C944246D12170F
ODR:            500Hz
OK

linux@ubuntu20:~/pyconfig$
```

### shell配置例程

#### 使用要求

* shell需求
  * bash > 4.0
* 测试平台
  * Ubuntu20.04
  * 树莓派4B  & JETSON NANO

#### 执行文件

```shell
bash hipnuc_con.sh /dev/ttyUSB0 hipnuc_config.yaml
```

示例：

```shell
linux@ubuntu20:~/shconfig$ bash hipnuc_con.sh /dev/ttyUSB0 hipnuc_config.yaml
IMU port:  /dev/ttyUSB0
CONFIG FILE: hipnuc_config.yaml
/dev/ttyUSB0:        37363 37367 37376 37380 37384 37389 37410 37418 37426 37435 
start
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
start
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
CONNECT SUCCESS
Detected baud rate: 921600
1.AT+EOUT=0

OK
CONNECT SUCCESS
2.AT+INFO
HI14R2-URT-000 1.5.1 build Apr  2 2024
MODE:           6 AIXS
UUID:           06C944246D12170F
ODR:            500Hz
OK
 
start config IMU
 
SCMD:   LOG IMU91 ONTIME 0.002
REV:	OK
SCMD:   SERIALCONFIG 921600
error
start
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
start
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
start
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
start
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
SCMD AT+EOUT=0 ERROR
start
SCMD AT+EOUT=0 ERROR
CONNECT SUCCESS
Detected baud rate: 921600
HI14R2-URT-000 1.5.1 build Apr  2 2024
MODE:           6 AIXS
UUID:           06C944246D12170F
ODR:            500Hz
OK
IXS
Done!!!
```

