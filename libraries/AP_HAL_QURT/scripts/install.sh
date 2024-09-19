#!/bin/bash

# Simple install script for voxl2 QURT 
# TO BE REAPLACED BY USING the upload option :
#       ./waf [build_vehicle] --upload
# Run from ardupilot repo root
# ./libraries/AP_HAL_QURT/scripts/install.sh 

# Requires three deb packages to be installed
# voxl-mavlink-server_1.4.1_arm64.deb
# modalai-slpi_1.1.19_arm64.deb
# libslpi-link_1.0.0_arm64.deb
# 

# Gather Development Files Needed
#-i  : specify ssh key to use
# ssh USERNAME@qurt.ardupilot.org -i /home/USERNAME/.ssh/KEY_NAME_rsa
#
# Grab the following folders using rsync
# rsync -chavzP --stats USERNAME@qurt.ardupilot.org:/opt/hexagon-sdk ~/Downloads
# rsync -chavzP --stats USERNAME@qurt.ardupilot.org:/opt/aarch64-sdk ~/Downloads
#
# Copy folders to the following
# cp -r ~/Downloads/hexagon-sdk/ /opt
# cp -r ~/Downloads/aarch64-sdk/ /opt
#
#
# If on Ubuntu 24.04 you will be unable to build due to the dependedcy on libncurses.so.5
# FIX is taken from here: https://community.localwp.com/t/installation-failed-in-ubuntu-24-04-lts/42579/3
#
#
# curl -O http://launchpadlibrarian.net/648013231/libtinfo5_6.4-2_amd64.deb
# sudo dpkg -i libtinfo5_6.4-2_amd64.deb
#
# curl -O http://launchpadlibrarian.net/648013227/libncurses5_6.4-2_amd64.deb
# sudo dpkg -i libncurses5_6.4-2_amd64.deb

# Must install ADB
# sudo apt install adb

# To BUILD
# ./waf configure --board QURT
# ./waf [vehicle]  IE [copter, plane, rover] 

# Get the Current ArduPilot Repo Directory
AP_DIR=$PWD 
QURT_BOARD_DIR="$AP_DIR/libraries/AP_HAL_QURT"




# Check for an adb device

# Stop the voxl-px4 service as it is installed and started by default on a fresh SDK install
adb shell systemctl disable voxl-px4
adb shell service voxl-px4 stop

# Make this an only if ps -ef | grep px4 returns something
adb shell pkill -9 px4

# Stop the ardupilot service if it is running
adb shell service voxl-ardupilot stop
adb shell pkill -9 ardu

# Push Service files
adb push $QURT_BOARD_DIR/ap_host/service/voxl-ardupilot.service /etc/systemd/system/
adb push $QURT_BOARD_DIR/ap_host/service/voxl-ardupilot /usr/bin/

# Push build
adb push $AP_DIR/build/QURT/ardupilot /usr/bin/
adb push $AP_DIR/build/QURT/bin/arducopter /usr/lib/rfsa/adsp/ArduPilot.so

# Enable the service and start ardupilot
adb shell systemctl enable voxl-ardupilot.service
adb shell systemctl start voxl-ardupilot
