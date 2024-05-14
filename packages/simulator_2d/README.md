# Simulator 2D

## Overview
The calculations are based on the [Ackeramnn model](../../doc/ackermann_vehicle.md).

## Simulator Engine API

| Name | Description | Arguments | Return type |
| --- | --- | --- | --- |
| **resetBase** | Sets the model center state | const geom::Pose& pose, double middle_steering, double linear_velocity | void |
| **resetMap** | Sets the model center state | const std::string& path | void |
| **eraseMap** | Sets the model center state |  | void |
| **getTruckState** | Sets the model center state |  | TruckState |
| **setBaseControl** | Sets the model center state | double velocity, double acceleration, double curvature | void |
| **setBaseControl** | Sets the model center state | double velocity, double curvature | void |
| **advance** | Sets the model center state | double seconds = 1.0 | void |

## Truck State API

| Name | Description | Type |
| --- | --- | --- |
| **time** | Simulation time | rclcpp::Time |
| **odomBasePose** | The model center pose | geom::Pose |
| **currentSteering** | Current steering | model::Steering |
| **targetSteering** | Target steering | model::Twist |
| **odomBaseLinearVelocity** | The model center linear velocity vector | geom::Vec2 |
| **baseAngularVelocity** | Angular velocity scalar | double |
| **lidarRanges** | Array of distances from lidar to obstacles (by rays) | const std::vector<float>& |
| **currentMotorRps** | Current motor RPS | double |
| **targetMotorRps** | Target motor RPS | double |
| **gyroAngularVelocity** | Angular velocity vector for IMU | geom::Vec3 |
| **accelLinearAcceleration** | Linear acceleration vector for IMU | geom::Vec3 |


## LiDAR calculations

Lidar is a sensor that determines the distance from itself to obstacles by pointing a laser at an object and measuring the time it takes for the reflected light to return to the receiver. The lidar message contains an array of ranges, which stores the distance from the i-th ray to the nearest obstacle.

![This is an image](doc/svg/lidar.svg)

The simulator engine generates a dummy message for the current localization of the car on the obstacle map. When loading the map, all obstacles are divided into segments. They are iterated when calculating an array of ranges. At each iteration, the polar angles of the lidar rays passing through the ends of this segment, rounded to the nearest value, are determined for the current segment. After that, all suitable rays are repeated in the nested loop, and the distance to the nearest obstacle is updated for each one.


## IMU calculations

![This is an image](doc/svg/imu.svg)

Rotation matrix – T

Tangential acceleration – $\vec{t}$

Centripetal acceleration – $\vec{c} = \vec{e} \cdot ⱱ'^2 \cdot C'$

Acceleration of free fall – $\vec{g}$

Linear acceleration – $T \cdot (\vec{t} + \vec{c} + \vec{g})$

Angular velocity – $T \cdot \vec{𝜔}$

The IMU message consists of linear acceleration and angular velocity vectors for the sensor center.

Tangential acceleration is directed tangentially to the trajectory of motion. Its x coordinate is equal to the acceleration scalar, y and z are equal to 0.

$\vec{e}$ is a unit vector co-directed to the $\vec{IO}$. The centripetal acceleration is equal to the $\vec{e}$ multiplied by the square of the scalar of the linear velocity and the curvature of the trajectory.

Linear acceleration is the vector sum of tangential, centripetal acceleration and free fall acceleration.

Since the model moves in a plane, in the $\vec{𝜔}$, the z coordinate is considered equal to the scalar of angular velocity, x and y are zero.