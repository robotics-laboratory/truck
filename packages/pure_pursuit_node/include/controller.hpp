#pragma once

#include <vector>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pure_pursuit_msgs/msg/command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visual_info.hpp"
#include "model/model.hpp"
#include "result.hpp"

namespace pure_pursuit {

enum class ControllerError {
    UNREACHEABLE_TRAJECTORY,
    IMPOSSIBLE_ARC,
    BROKEN_FORMULA_FOR_ACCELERATION
};

inline std::string error_to_string(ControllerError e) {
    switch (e) {
    case ControllerError::UNREACHEABLE_TRAJECTORY:
        return "Can not find the target point";
    case ControllerError::IMPOSSIBLE_ARC:
        return "Can not build the arc";
    case ControllerError::BROKEN_FORMULA_FOR_ACCELERATION:
        return "Broken formula for acceleration";
    default:
        return "Unknown error";
    }
}

using ControllerResult = Result<pure_pursuit_msgs::msg::Command, ControllerError>;

class Controller {
private:
    model::Model model;

public:
    Controller(const model::Model &model) : model{model} {}
    ControllerResult get_motion(
        const nav_msgs::msg::Odometry &odometry,
        const std::vector<geometry_msgs::msg::PoseStamped> &path,
        VisualInfo *visual_info = nullptr);
};

};  // namespace pure_pursuit
