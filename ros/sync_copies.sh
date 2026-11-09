#!/bin/sh
# Copy the C decoder and serial sources into the ROS 2 package, which builds
# on its own (bloom releases each package directory separately), and its
# message into the ROS 1 package.
set -eu
cd "$(dirname "$0")/.."
dest=ros/hipnuc_imu/hipnuc
rm -rf "$dest"
mkdir -p "$dest"
for file in hipnuc_data.c hipnuc_data.h hipnuc_decoder.c hipnuc_decoder.h hipnuc_nmea.c hipnuc_nmea.h \
    hipnuc_internal.h; do
    cp "c/hipnuc/$file" "$dest/"
done
for file in hipnuc_serial.c hipnuc_serial.h hipnuc_serial_io.c hipnuc_serial_io.h; do
    cp "c/serial/$file" "$dest/"
done
mkdir -p ros/ros1/src/hipnuc_imu/msg
cp ros/hipnuc_imu/msg/HipnucImu.msg ros/ros1/src/hipnuc_imu/msg/
