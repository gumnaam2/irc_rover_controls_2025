#!/usr/bin/env bash

BIT_RATE=500000

if ! ifconfig | grep -q "can0" ; then 
    sudo ip link set can0 type can bitrate "$BIT_RATE"
    sudo ifconfig can0 up
    sudo ifconfig can0 txqueuelen 65536
fi


