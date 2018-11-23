#!/bin/bash
argc=$#
i=0
make -j8
if(($argc<1))
then
    echo "Flash sink and sender image"
    make udp-sink.upload PORT=/dev/ttyUSB0 &
    make udp-sender.upload PORT=/dev/ttyUSB1 
    exit
fi

if [ "$1" == "0" ]
then
    echo "Flash sinkimage"
    make udp-sink.upload PORT=/dev/ttyUSB0 
else
    echo "Flash sender image"
    for((i=1; i<=$argc; i++))
    do
    	make udp-sender.upload PORT=/dev/ttyUSB$i &
    	sleep 1
    done
fi