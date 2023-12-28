#!/bin/bash

# Default baud rate
default_baudrate=115200

# Function to display usage information
usage() {
    echo "Usage: $0 <file> <tty> [baudrate] [blhost_path]"
    echo "       baudrate and blhost_path are optional"
    echo "       default baudrate is $default_baudrate"
    echo "       default blhost_path is ./blhost/amd64/blhost"
}

# Check if at least two arguments are provided
if [ "$#" -lt 2 ]; then
    usage
    exit 1
fi

# Assign arguments to variables
file=$1
tty=$2
baudrate=${3:-$default_baudrate}
blhost_cmd=${4:-"./blhost/amd64/blhost"}

# Check if the firmware file exists
if [ ! -f "$file" ]; then
    echo "Error: File '$file' not found!"
    exit 1
fi

# Check if the TTY device exists
if [ ! -c "$tty" ]; then
    echo "Error: TTY device '$tty' not found!"
    exit 1
fi

# Check if blhost command is executable
if [ ! -x "$blhost_cmd" ]; then
    echo "Error: blhost command not executable or not found at '$blhost_cmd'!"
    exit 1
fi

# Configure the serial port
stty -F $tty raw speed $baudrate -echo min 0 time 5 || {
    echo "Error: Failed to configure TTY device $tty"
    exit 1
}

# Send AT command and wait for 30 ms
echo -ne "AT+RST\r\n" > $tty
sleep 0.03

# Send binary data 5A A6
echo "5AA6" | xxd -r -ps > $tty || {
    echo "Error: Failed to send binary data to $tty"
    exit 1
}

# Use the blhost tool to download the firmware
$blhost_cmd -p $tty,$baudrate get-property 1 || exit 1
$blhost_cmd -p $tty,$baudrate -- flash-image $file erase || exit 1
$blhost_cmd -p $tty,$baudrate reset || exit 1
