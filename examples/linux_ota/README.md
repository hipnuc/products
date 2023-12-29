# Linux下模块固件升级

## 支持环境

* ubuntu(amd64)
* 树莓派4B

## 升级步骤

1. 确认模块可以正常工作，确认模块串口, 如ttyUSB0等。
2.  将固件升级文件(.hex)放到本目录下.
3. 固件升级脚本用法

```shell
Usage: ./firmware_update.sh <file> <tty> [baudrate] [blhost_path]
       blhost_path are optional
       default baudrate is 115200
       default blhost_path is ./blhost/amd64/blhost
```

例:

```shell
$ chmod +x firmware_update.sh
$ sudo ./firmware_update.sh ./imu_image.hex /dev/ttyUSB0 115200
```

成功执行后会有如下log，返回`Successful generic response to command 'reset'` 即表示下载成功

```
Ping responded in 1 attempt(s)
Inject command 'get-property'
Response status = 0 (0x0) Success.
Response word 1 = 1258357760 (0x4b010400)
Current Version = K1.4.0
Ping responded in 1 attempt(s)
Inject command 'flash-image'
Successful generic response to command 'flash-erase-region'
Wrote 90712 bytes to address 0x8004000
Successful generic response to command 'write-memory'
(1/1)100% Completed!
Successful generic response to command 'write-memory'
Response status = 0 (0x0) Success.
Ping responded in 1 attempt(s)
Inject command 'reset'
Successful generic response to command 'reset'
Response status = 0 (0x0) Success.
```
