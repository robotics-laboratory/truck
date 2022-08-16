#!/bin/bash

cd /opt/gzweb

source /usr/share/gazebo/setup.sh
npm run deploy --- -m local
npm run deploy --- -t