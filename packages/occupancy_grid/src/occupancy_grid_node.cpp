#include "occupancy_grid/occupancy_grid_node.h"

#include "common/math.h"

#include <cstdint>
#include <functional>

namespace truck::occupancy_grid {

OccupancyGridNode::OccupancyGridNode() : Node("occupancy_grid") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    RCLCPP_INFO(this->get_logger(), "qos %d", qos);

    params_ = {this->declare_parameter("resolution", 0.1), this->declare_parameter("radius", 20.0)};

    RCLCPP_INFO(this->get_logger(), "resolution: %.2fm", params_.resolution);
    RCLCPP_INFO(this->get_logger(), "radius: %.2fm", params_.radius);

    scan_slot_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar/scan",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&OccupancyGridNode::handleLaserScan, this, std::placeholders::_1));

    grid_signal_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/grid", 10);
}

void OccupancyGridNode::handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    nav_msgs::msg::OccupancyGrid grid;

    const auto [resolution, radius] = params_;
    const auto cell_radius = ceil<int>(radius / resolution);
    const auto cell_num = 2 * cell_radius + 1;

    grid.header = scan->header;
    grid.info.resolution = resolution;
    grid.info.width = cell_num;
    grid.info.height = cell_num;

    grid.info.origin.position.x = -radius;
    grid.info.origin.position.y = -radius;
    grid.info.origin.position.z = 0;

    grid.info.origin.orientation.x = 0;
    grid.info.origin.orientation.y = 0;
    grid.info.origin.orientation.z = 0;
    grid.info.origin.orientation.w = 1;

    grid.data.resize(cell_num * cell_num);

    for (size_t k = 0; k < scan->ranges.size(); ++k) {
        const double range = scan->ranges[k];

        const bool valid_range = (scan->range_min <= range) && (range <= scan->range_max);
        if (!valid_range) {
            continue;
        }

        const double angle = scan->angle_min + k * scan->angle_increment;

        const double x = range * std::cos(angle) + radius;
        const double y = range * std::sin(angle) + radius;

        const auto i = clamp(static_cast<int>(x / resolution), 0, cell_num - 1);
        const auto j = clamp(static_cast<int>(y / resolution), 0, cell_num - 1);

        grid.data[i + j * cell_num] = 100;
    }

    grid_signal_->publish(grid);
}

}  // namespace truck::occupancy_grid