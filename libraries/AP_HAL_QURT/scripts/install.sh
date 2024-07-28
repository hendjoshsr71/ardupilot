#!/bin/bash

# Simple install script for voxl2 QURT 
# TO BE REAPLACED BY USING the upload option :
#       ./waf [build_vehicle] --upload
# Run from ardupilot repo root

# Requires three deb packages to be installed
# voxl-mavlink-server_1.4.1_arm64.deb
# modalai-slpi_1.1.19_arm64.deb
# libslpi-link_1.0.0_arm64.deb
# 

# Get the Current ArduPilot Repo Directory
AP_DIR=$PWD 
QURT_BOARD_DIR="$AP_DIR/libraries/AP_HAL_QURT"




# Check for an adb device

# Stop the voxl-px4 service as it is installed and started by default on a fresh SDK install
adb shell systemctl disable voxl-px4
adb shell service voxl-px4 stop

# Stop the ardupilot service if it is running
adb shell service voxl-ardupilot stop

# Push Service files
adb push $QURT_BOARD_DIR/ap_host/service/voxl-ardupilot.service /etc/systemd/system/
adb push $QURT_BOARD_DIR/ap_host/service/voxl-ardupilot /usr/bin/

# Push build
adb push $AP_DIR/build/QURT/ardupilot /usr/bin/
adb push $AP_DIR/build/QURT/bin/arducopter /usr/lib/rfsa/adsp/ArduPilot.so

# Enable the service and start ardupilot
adb shell systemctl enable voxl-ardupilot.service
adb shell systemctl start voxl-ardupilot
