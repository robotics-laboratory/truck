#!/bin/bash -xe

DEVICE="${DEVICE:-can0}"
SPEED="${SPEED:-500000}"
PORT="${PORT:-1000}"

modprobe -a mttcan vcan can-gw
ip link set $DEVICE down
ip link set $DEVICE type can bitrate $SPEED
ip link set $DEVICE up
cannelloni -I $DEVICE -l $PORT -C s -p