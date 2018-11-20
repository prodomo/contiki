#!/bin/bash

echo "compiling..."
make TARGET=cc2538dk

# echo "uploading..."
# sshpass -psakimaru1101 scp udp-sender.bin admin@192.168.1.250:/volume7/WorkingData/contiki/firmware/sender_`date +%m%d%H%M%S`.bin