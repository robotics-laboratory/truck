#!/bin/bash

cd /opt/gzweb

source /usr/share/gazebo/setup.sh

npm run deploy --- -m
npm run deploy --- -t
