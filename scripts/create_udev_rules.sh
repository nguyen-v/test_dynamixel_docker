#!/bin/bash

echo "remap the device serial port(ttyUSBX) to dynamixel"
echo "dynamixel usb connection as /dev/dynamixel , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy dynamixel.rules to  /etc/udev/rules.d/"
sudo cp scripts/dynamixel.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "