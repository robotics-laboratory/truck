#pragma once

#include <vector>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pure_pursuit_msgs/msg/command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visual_info.hpp"
#include "model.hpp"
#include "result.hpp"

namespace pure_pursuit {

using ControllerResult = Result<pure_pursuit_msgs::msg::Command, const char *>;

class Controller {
private:
    model::Model params;

public:
    Controller(const model::Model &params) : params{params} {}
    ControllerResult get_motion(
        const nav_msgs::msg::Odometry &odometry,
        const std::vector<geometry_msgs::msg::PoseStamped> &path,
        VisualInfo *visual_info = nullptr);
};

};  // namespace pure_pursuit
