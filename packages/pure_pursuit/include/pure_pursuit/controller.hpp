#pragma once

#include <vector>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pure_pursuit_msgs/msg/command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pure_pursuit/visual_info.hpp"
#include "model/model.hpp"
#include "pure_pursuit/result.hpp"

namespace pure_pursuit {

enum class ControllerError {
    UNREACHEABLE_TRAJECTORY,
    IMPOSSIBLE_ARC
};

inline std::string errorToString(ControllerError e) {
    switch (e) {
    case ControllerError::UNREACHEABLE_TRAJECTORY:
        return "Can not find the target point";
    case ControllerError::IMPOSSIBLE_ARC:
        return "Can not build the arc";
    default:
        return "Unknown error";
    }
}

struct ControllerResultData {
    pure_pursuit_msgs::msg::Command cmd;
    std::optional<VisualInfo> visual_info;
};

using ControllerResult = Result<ControllerResultData, ControllerError>;

class Controller {
private:
    model::Model model;

public:
    Controller(const model::Model &model) : model{model} {}
    ControllerResult getMotion(
        const nav_msgs::msg::Odometry &odometry,
        const std::vector<geometry_msgs::msg::PoseStamped> &path,
        bool visual_info_required = 0);
};

};  // namespace pure_pursuit
