#!/bin/bash

sudo cp hipnuc_imu_serial.rules /etc/udev/rules.d/
service udev reload
service udev restart