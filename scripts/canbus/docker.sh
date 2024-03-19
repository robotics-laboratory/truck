#!/bin/bash -xe

DEVICE="${DEVICE:-vcan0}"
PORT="${PORT:-1000}"
HOST=$(ip route | grep default | cut -d " " -f 3)

ip link add $DEVICE type vcan || true
ip link set $DEVICE up
cannelloni -I $DEVICE -R $HOST -r $PORT -C c
