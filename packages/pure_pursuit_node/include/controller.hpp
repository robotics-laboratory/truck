#pragma once

#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pure_pursuit_msgs/msg/command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visual_info.hpp"
#include "model.hpp"

namespace pure_pursuit {

class Controller {
private:
    model::Model params;

public:
    Controller(const model::Model &params) : params{params} {}
    pure_pursuit_msgs::msg::Command get_motion(
        const nav_msgs::msg::Odometry &odometry,
        const std::vector<geometry_msgs::msg::PoseStamped> &path,
        VisualInfo *visual_info = nullptr);
};

};  // namespace pure_pursuit
