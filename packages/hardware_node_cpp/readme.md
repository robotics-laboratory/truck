# Harware Node

## Overview

This package provides a ROS node to control robot movement on real (not simulated) hardware. More specifically, it communicates with ODrive (BLDC motor controller) and Teensy MCU (manages servo motors for steering) using CAN

## Paramethers

- `model_config` — path to `model.yaml` file, used by `model::Model` class ([see here for details](../model/include/model/model.h))
- `steering_config` — path to `steering.csv` file ([see here for details](hardware_node/teensy.py))
- `odrive_timeout` (default: 250 ms) — [odrive watchog](https://docs.odriverobotics.com/v/0.5.4/getting-started.html#watchdog-timer) timeout (milliseconds). Adjust to be slightly higher than the interval between control messages (`/control/command` topic).
- `status_report_rate` (default: 1 Hz) — rate of messages in `/hardware/status` topic.
- `telemetry_report_rate` (default: 20 Hz) — rate of messages in `/hardware/telemetry` topic.
- `odrive_axis` (default: "axis1") — name of odrive axis with connected motor.

## Topics

### Input:
- `/control/command` — current velocity and steering target ([see here](../truck_msgs/msg/Control.msg)).
- `/control/mode` — current mode of operation ([see here](../truck_msgs/msg/ControlMode.msg)).
    - `OFF` = motion disabled (odrive state = IDLE, motor coils off).
    - `REMOTE` = robot is controlled via remote (gamepad).
    - `AUTO` = robot is controlled programmatically.

### Output:
- `/hardware/status` — global status of hardware ([see here](../truck_msgs/msg/HardwareStatus.msg)). In case of odrive failure, message will contain list of internal error codes. Published with a regular rate = `telemetry_report_rate`, also published immediatly after odrive error occurs.
- `/hardware/telemetry` — odrive internal metrics, useful for diagnostic ([see here](../truck_msgs/msg/HardwareTelemetry.msg)). Published with a relatively high rate = `telemetry_report_rate`.
