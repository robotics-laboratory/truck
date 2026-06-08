#!/bin/bash
ros2 bag record \
  --compression-mode file \
  -o bags/$1 \
  /control/command \
  /control/input \
  /control/mode \
  /grid \
  /hardware/debug_status \
  /left/scan \
  /left/scan_filtered \
  /merged/scan \
  /motion/command \
  /motion/pure_pursuit/status \
  /motion/trajectory \
  /odom \
  /right/scan \
  /right/scan_filtered \
  /tf \
  /tf_static \
  /visualization/arc \
  /visualization/ego \
  /visualization/ego/track \
  /visualization/navigation/mesh \
  /visualization/navigation/route \
  /visualization/trajectory \
  /visualisation/waypoints \
  /waypoints
