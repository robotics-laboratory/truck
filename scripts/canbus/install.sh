#!/bin/bash -xe

cp canbridge.sh /usr/bin
cp canbridge.service /etc/systemd/system
systemctl enable --now canbridge
systemctl reload-or-restart canbridge
