#!/bin/bash

# 默认波特率
default_baudrate=115200

# 显示使用信息的函数
usage() {
    echo "Usage: $0 <file> <tty> [baudrate] [blhost_path]"
    echo "       baudrate and blhost_path are optional"
    echo "       default baudrate is $default_baudrate"
    echo "       blhost_path is determined based on system architecture if not provided"
}

# 检查是否提供了至少两个参数
if [ "$#" -lt 2 ]; then
    usage
    exit 1
fi

# 分配参数到变量
file=$1
tty=$2
baudrate=${3:-$default_baudrate}

# 基于处理器架构确定 blhost 路径
arch=$(uname -m)
echo "Detected CPU architecture: $arch"
if [ "$#" -lt 4 ]; then
    case $arch in
        "x86_64")
            blhost_cmd="./blhost/amd64/blhost"
            ;;
        "armv7l"|"aarch64")
            blhost_cmd="./blhost/armv7l/blhost"
            ;;
        *)
            echo "Error: Unsupported architecture: $arch"
            exit 1
            ;;
    esac
else
    blhost_cmd=$4
fi

# 检查固件文件是否存在
if [ ! -f "$file" ]; then
    echo "Error: File '$file' not found!"
    exit 1
fi

# 检查 TTY 设备是否存在
if [ ! -c "$tty" ]; then
    echo "Error: TTY device '$tty' not found!"
    exit 1
fi

# 检查 blhost 命令是否可执行
if [ ! -x "$blhost_cmd" ]; then
    echo "Error: blhost command not executable or not found at '$blhost_cmd'!"
    exit 1
fi

# 配置串口
stty -F $tty raw speed $baudrate -echo min 0 time 5 || {
    echo "Error: Failed to configure TTY device $tty"
    exit 1
}

# 发送 AT 命令并等待 30 毫秒
echo -ne "AT+RST\r\n" > $tty
sleep 0.03

# 发送二进制数据 5A A6
echo "5AA6" | xxd -r -ps > $tty || {
    echo "Error: Failed to send binary data to $tty"
    exit 1
}

# 检查 blhost 命令是否可执行，如果不是则尝试添加执行权限
if [ ! -x "$blhost_cmd" ]; then
    echo "blhost command not executable. Trying to set execute permission..."
    chmod +x "$blhost_cmd" || {
        echo "Failed to set execute permission on $blhost_cmd. Please run 'sudo chmod +x $blhost_cmd' manually."
        exit 1
    }
fi

# 使用 blhost 工具下载固件
$blhost_cmd -p $tty,$baudrate get-property 1 || exit 1
$blhost_cmd -p $tty,$baudrate -- flash-image $file erase || exit 1
$blhost_cmd -p $tty,$baudrate reset || exit 1

# 等待 1 秒以确保设备重置完成
sleep 1

# 在固件升级后发送 FRESET 命令
echo -ne "FRESET\r\n" > $tty
sleep 0.03  # 等待设备处理命令

echo "Firmware updated and device reset to factory settings."
