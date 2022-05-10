#pragma once

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_ros/node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sdf/sdf.hh>

#include <memory>
#include <string>

namespace gazebo {

class OccupancyGridPlugin : public WorldPlugin {
  public:
    OccupancyGridPlugin() : WorldPlugin() {}

    virtual ~OccupancyGridPlugin() = default;

    void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

    nav_msgs::msg::OccupancyGrid GetOccypancyGrid();

    void OnWorldUpdate();

    std::string name_ = "OccupancyGridPlugin";
    std::string grid_frame_id_ = "odom";
    std::string grid_topic_ = "/occupancy_grid";

    double period_ = 0.1;
    double resolution_ = 0.1;
    double size_ = 10.0;

    gazebo::common::Time last_update_time_ = {};

    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr world_update_;

    gazebo_ros::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
};

}  // namespace gazebo