#!/bin/sh
# /usr/local/bin/btslave.sh
# Script to start bluetooth slave serial port profile with login shell
# Pairing is done separately, e.g. with bluetoothctl or gnome-bluetooth
# ModemManager has been disabled to prevent interference:
#	  systemctl disable ModemManager
# Run as startup service:
#	  systemctl link /usr/local/etc/btslave.service
#	  systemctl start btslave.service
#
# Christopher A. Chavez
# 11/15/2015
#
# References:
# 
# hciconfig command example
# http://raspberrypi.stackexchange.com/a/19865
# 
# rfcomm + agetty slave example
# http://unix.stackexchange.com/a/153651
#
# for disabling ModemManager in systemd
# http://askubuntu.com/a/612646
#
# for writing and registering systemd boot script
# http://unix.stackexchange.com/a/47715

# configure bluetooth device (assume hci0) with page and inquiry scan
hciconfig hci0 piscan

# Create port /dev/rfcomm0
# wait for connection, attach to login shell with baud 115200
# watch--will restart if reconnected
rfcomm watch /dev/rfcomm0 1 /sbin/agetty rfcomm0 115200 linux
