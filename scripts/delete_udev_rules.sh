#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  dynamixel"
echo "sudo rm   /etc/udev/rules.d/dynamixel.rules"
sudo rm   /etc/udev/rules.d/dynamixel.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"