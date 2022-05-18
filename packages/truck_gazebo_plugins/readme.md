# Truck plugins

## OdometryPlugin

### Publish:
- /odom ([nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)) – position of selected link in odometry world.
- **/tf** [[nav_msgs/Odometry](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Transform.html)] – transform from global world to odometric (identical in case of simulation).

### Using
```
<plugin name="OdometryPlugin" filename="libtruck_gazebo_plugins.so">
    <!--  name of link (required) -->
    <link_name>base_link</link_name>

    <!-- period of publishing, default 0.0 (as soon as possible) -->
    <period>0.1</period>

    <!--  offset of base point in frame of link, default without offset -->
    <xyz_offsets>10 0 0</xyz_offsets>
</plugin>
```