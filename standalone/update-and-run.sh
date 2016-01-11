git pull
avrdude -C /usr/share/arduino/hardware/tools/avrdude.conf -v -p m328p -c alamode -b 115200 -P /dev/ttyS0  -D -Uflash:w:standalone/standalone.ino.with_bootloader.standard.hex
screen /dev/ttyS0 9600
