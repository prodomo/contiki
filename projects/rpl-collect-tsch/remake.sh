#! /bin/sh
make clean
make -j
if [ "$1" == "sink" ]; then
	make udp-sink.upload port=/dev/tty.SLAB_USBtoUART
else
	make udp-sender.upload port=/dev/tty.SLAB_USBtoUART
fi
miniterm.py /dev/tty.SLAB_USBtoUART 115200
