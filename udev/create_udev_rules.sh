#!/bin/bash

WORKING_DIR=$(dirname $(readlink -f $0))
cd $WORKING_DIR

echo "remap the device serial ports of mowbot"
echo "start copy mowbot_udev.rules to /etc/udev/rules.d/"
sudo cp ${WORKING_DIR}/mowbot_udev.rules /etc/udev/rules.d/mowbot_udev.rules
echo -e "\nRestarting udev\n"
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish"