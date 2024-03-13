# AckermannVehicle
Implements dummy (no slippage) [ackermann model](../../doc/ackermann_vehicle.md).
1. Truck has *constant* steering velocity and limited torque. Each wheel is moving with separate motor in bang-bang manner. Stopping point is determined with tolerance.
2. Rear wheels has one motor with fixed torque and gear ratio is known. It is assumed that no any slippage. Angular velocity is evaluated for required linear velocity and curvature. Velocity is controlled with PD regulator.

## Config
```
<plugin name='AckermannControlPlugin' filename='libtruck_gazebo_ackermann_model.so'>
  <config_path>/truck/packages/model/config/model.yaml</config_path>
  <steering>
    <!-- stopping point error -->
    <error>0.03</error>
    <!-- steering velocity in degrees -->
    <velocity>120</velocity>
    <!-- motor torque -->
    <torque>0.5</torque>
    <left_joint>left_steering_joint</left_joint>
    <right_joint>right_steering_joint</right_joint>
  </steering>
  <rear>
    <!-- controller params of reguqlators -->
    <pd>0.1 0.0</pd>
    <!-- motor torque -->
    <torque>10.0</torque>
    <left_joint>left_rear_axle</left_joint>
    <right_joint>right_rear_axle</right_joint>
  </rear>
</plugin>
```

## Odometry
Publish pose of model in odometric frame and transform `world` -> `odom`.


## Config
```
 <plugin name="OdometryPlugin" filename="libtruck_gazebo_odometry.so">
  <!-- publish pose of link fram -->
  <link_name>rplidar_s2::lidar_base</link_name>
  <!-- period of publication (milliseconds) -->
  <period>50</period>
  <!-- offset in link frame -->
  <xyz_offset>0.0 0.0 0.0</xyz_offset>
 </plugin>
```

## Topics
### Output
- `/tf` [[tf2_msgs/TFMessage]](http://docs.ros.org/en/api/tf2_msgs/html/msg/TFMessage.html) - transform `world` -> `odom` (identical).
- `/odom` [[nsv2_msgs/Odometry]](http://docs.ros.org/en/api/nav2_msgs/html/msg/Odometry.html) - pose of link in odometric world.