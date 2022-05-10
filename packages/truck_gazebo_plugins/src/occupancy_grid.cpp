#include "truck_gazebo_plugins/occupancy_grid.h"

#include <gazebo_ros/conversions/builtin_interfaces.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>

namespace gazebo {

void OccupancyGridPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);

    grid_topic_ = sdf->Get<std::string>("grid_topic", grid_topic_).first;
    period_ = sdf->Get<double>("period", period_).first;
    resolution_ = sdf->Get<double>("cell_size", resolution_).first;
    size_ = sdf->Get<double>("cell_size", size_).first;

    grid_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, 10);
    RCLCPP_INFO(node_->get_logger(), "Publish grid on [%s]", grid_topic_.c_str());

    // Listen to the update event (broadcast every simulation iteration)
    world_update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&OccupancyGridPlugin::OnWorldUpdate, this));
}

nav_msgs::msg::OccupancyGrid OccupancyGridPlugin::GetOccypancyGrid() {
    nav_msgs::msg::OccupancyGrid result;

    const double truck_x = 0;
    const double truck_y = 0;

    const size_t width = size_ / resolution_;
    const size_t height = size_ / resolution_;

    const double min_x = truck_x - size_ / 2;
    const double min_y = truck_x - size_ / 2;

    const double max_x = min_x + width * resolution_;
    const double max_y = min_y + height * resolution_;

    result.header.frame_id = grid_frame_id_;
    result.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(world_->SimTime());

    result.info.map_load_time = rclcpp::Time(0);
    result.info.resolution = resolution_;
    result.info.width = width;
    result.info.height = height;

    result.info.origin.position.x = min_x;
    result.info.origin.position.y = min_y;
    result.info.origin.position.z = 0;

    result.info.origin.orientation.x = 0;
    result.info.origin.orientation.y = 0;
    result.info.origin.orientation.z = 0;
    result.info.origin.orientation.w = 1;

    result.data.resize(width * height);
    std::fill(result.data.begin(), result.data.end(), 0);

    gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
    engine->InitForThread();

    gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    const double z = 0.1;
    const double max_distance = size_ / std::sqrt(2);

    double dist = 0;
    std::string entity_name;

    for (double angle = 0; angle < 2 * 3.14; angle += 0.001) {
        const double cos_a = std::cos(angle);
        const double sin_a = std::sin(angle);

        double x = max_distance * cos_a + truck_x;
        double y = max_distance * sin_a + truck_y;

        ray->SetPoints({truck_x, truck_y, z}, {x, y, z});
        ray->GetIntersection(dist, entity_name);

        if (entity_name.empty()) {
            continue;
        }

        x = dist * cos_a + truck_x;
        y = dist * sin_a + truck_y;

        if (min_x > x || max_x < x || min_y > y || max_y < y) {
            continue;
        }

        const size_t row = (y - min_y) / resolution_;
        const size_t col = (x - min_x) / resolution_;
        const size_t index = row * width + col;

        result.data[index] = 100;
    }

    return result;
}

void OccupancyGridPlugin::OnWorldUpdate() {
    const common::Time now = world_->SimTime();
    if (now - last_update_time_ < period_) {
        return;
    }

    grid_publisher_->publish(GetOccypancyGrid());
    last_update_time_ = now;
}

GZ_REGISTER_WORLD_PLUGIN(OccupancyGridPlugin)

}  // namespace gazebo