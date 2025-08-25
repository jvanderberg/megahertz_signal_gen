#!/bin/bash

DEVICE="/dev/tty.usbmodem101"
BAUD="115200"

while true; do
    if [ -e "$DEVICE" ]; then
        echo "Launching screen on $DEVICE..."
        screen "$DEVICE" "$BAUD"
        echo "Screen session ended. Retrying..."
    else
        echo "Device not found. Retrying in 1s..."
    fi
    sleep 1
done

