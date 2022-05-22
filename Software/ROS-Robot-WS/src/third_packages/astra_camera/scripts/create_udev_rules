#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc to facilitate bringing"
echo "up the astra usb connection as /dev/astra*"
echo ""

sudo cp `rospack find astra_camera`/56-orbbec-usb.rules /etc/udev/rules.d


echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
#sudo udevadm trigger --action=change
