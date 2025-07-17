#!/bin/bash

sudo rm /etc/udev/rules.d/hipnuc_imu_serial.rules
service udev reload
service udev restart