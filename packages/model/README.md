# Model

## Overview
[Ackeramnn model](../../doc/ackermann_vehicle.md) parameters and some simple calculations.

![This is an image](../../doc/svg/ackermann_vehicle.svg)

# Model TF
## Overview
Publishes static transformation of models (listed in yaml file). This is the simplest analogue of `static_transform_publisher` and `joint_state_publisher`, but for our own purpouses (use sdf model format).

## Parameters
- `model_path` — path to yaml file with model config (including transform).
- `period` — period of transforms publishing (milliseconds).

## Yaml format
```
- frame_id: "from"
  child_frame_id: "to"
  translation: {x: 0.0, y: 0.0, z: 0.0}
  rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

### Output:
- `/tf` [[tf2_msgs/TFMessage]](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) - transforms 
