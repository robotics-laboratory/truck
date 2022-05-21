#pragma once

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_ros/node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>

#include <memory>
#include <string>
#include <mutex>

namespace gazebo {

typedef ignition::math::Vector3d vector3d;
auto GetPhysicsPtr = std::mem_fn(&gazebo::physics::World::Physics);

class OccupancyGridPlugin : public WorldPlugin {
  public:
    OccupancyGridPlugin() : WorldPlugin() {}

    virtual ~OccupancyGridPlugin() = default;

    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    bool worldCellIntersection(const vector3d& cell_center, const double cell_length,
                              gazebo::physics::RayShapePtr ray);

    void CreateOccupancyMap();

    static void cell2world(unsigned int cell_x, unsigned int cell_y,
                         double map_size_x, double map_size_y, double map_resolution,
                         double& world_x, double &world_y);

    static void world2cell(double world_x, double world_y,
                         double map_size_x, double map_size_y, double map_resolution,
                         unsigned int& cell_x, unsigned int& cell_y);

    static bool cell2index(int cell_x, int cell_y,
                         unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& map_index);

    static bool index2cell(int index, unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& cell_x, unsigned int& cell_y);

    void topic_callback(nav_msgs::msg::Odometry::UniquePtr odometry_)
    {
      std::lock_guard<std::mutex> guard(odometry_mutex_);
      last_odometry_ = *odometry_;
    }

    nav_msgs::msg::OccupancyGrid GetOccypancyGrid();

    void OnWorldUpdate();

    std::string name_ = "OccupancyGridPlugin";
    std::string grid_frame_id_ = "odom";
    std::string grid_topic_ = "/occupancy_grid";

    double period_ = 0.1;
    double map_height_;
    double map_resolution_;
    double map_size_x_;
    double map_size_y_;
    double init_robot_x_;
    double init_robot_y_;

    gazebo::common::Time last_update_time_ = {};
    std::mutex odometry_mutex_;
    nav_msgs::msg::Odometry odometry_;
    nav_msgs::msg::Odometry last_odometry_;

    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr world_update_;

    gazebo_ros::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_listener_;
};
}  // namespace gazebo