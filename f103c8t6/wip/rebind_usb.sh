#!/bin/bash

set -e

DEV="1-5.1"
echo "Unbinding USB device $DEV..."
echo "$DEV" | sudo tee /sys/bus/usb/drivers/usb/unbind >/dev/null
sleep 0.2
echo "Rebinding USB device $DEV..."
echo "$DEV" | sudo tee /sys/bus/usb/drivers/usb/bind >/dev/null
sleep 0.5
echo "USB device:"
lsusb -d 0483:5740
echo
echo "TTY:"
ls -l /dev/ttyACM* 2>/dev/null || true

#echo '1-5.1' | sudo tee /sys/bus/usb/drivers/usb/unbind
#echo '1-5.1' | sudo tee /sys/bus/usb/drivers/usb/bind
#udevadm info -q path -n /dev/ttyACM1

#PICOCOM_DEV=$(readlink -f /dev/serial/by-id/usb-STMicroelectronics_Virtual_COM_Port_*)
#picocom "$PICOCOM_DEV"
#sudo uhubctl -p 1 -a off

