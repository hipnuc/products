#!/bin/sh
# Copy the C decoder and serial sources into the ROS 2 package, which builds
# on its own (bloom releases each package directory separately), and its
# message into the ROS 1 package.
set -eu
cd "$(dirname "$0")/.."
dest=ros/hipnuc_imu/hipnuc
rm -rf "$dest"
mkdir -p "$dest"
for file in hipnuc_dec.c hipnuc_dec.h nmea_dec.c nmea_dec.h hipnuc_sample.c hipnuc_sample.h; do
    cp "c/hipnuc/$file" "$dest/"
done
for file in hipnuc_serial.c hipnuc_serial.h serial_io.c serial_io.h; do
    cp "c/serial/$file" "$dest/"
done
mkdir -p ros/ros1/src/hipnuc_imu/msg
cp ros/hipnuc_imu/msg/HipnucImu.msg ros/ros1/src/hipnuc_imu/msg/
