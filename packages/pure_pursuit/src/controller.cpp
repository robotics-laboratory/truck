#include "pure_pursuit/controller.hpp"
#include "pure_pursuit/speed_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "geom/vector.hpp"
#include "geom/arc.hpp"

using pure_pursuit_msgs::msg::Command;
using namespace geometry_msgs::msg;
using visualization_msgs::msg::Marker;
using geom::Vec2d;

namespace {

double quaternoin_to_flat_angle(const Quaternion& q) {
    return std::copysign(2 * std::acos(q.w), q.z);
}

inline double ros_time_to_seconds(const rclcpp::Time& t) { return t.seconds(); }

};  // namespace

namespace pure_pursuit {

ControllerResult Controller::getMotion(const nav_msgs::msg::Odometry& odometry,
                                        const std::vector<PoseStamped>& path,
                                        bool visual_info_required) {
    std::optional<VisualInfo> visual_info;
    if (visual_info_required) {
        visual_info.emplace();
        visual_info->addPoint(Vec2d(odometry.pose.pose.position), 0.1, 1, 0, 0);
    }
    auto& position = odometry.pose.pose.position;
    auto it = std::find_if(path.rbegin(), path.rend(), [position = Vec2d(position), this](const PoseStamped& p) {
        return geom::dist(Vec2d(p.pose.position), position) <= model.lookahead_distance;
    });
    if (it == path.rend()) return ControllerError::UNREACHEABLE_TRAJECTORY;
    if (visual_info_required) {
        for (auto& x : path) {
            visual_info->addPoint(Vec2d(x.pose.position), 0.1, 0, 0, 1);
        }
    }
    Vec2d p0(position);
    Vec2d p(it->pose.position);
    Vec2d direction{1, 0};
    direction = direction.rotate(quaternoin_to_flat_angle(odometry.pose.pose.orientation));

    auto trajectory = geom::Arc::fromTwoPointsAndTangentalVector(p0, p, direction, 1e-3);

    if (!trajectory) {
        return ControllerError::IMPOSSIBLE_ARC;
    }

    if (visual_info_required) {
        constexpr int points = 50;
        for (int i = 1; i < points; ++i) {
            visual_info->addPoint(trajectory->getPoint(static_cast<double>(i) / points), 0.05, 0, 1, 0);
        }
    }

    double dist = trajectory->getLength();

    Vec2d velocity_vector{odometry.twist.twist.linear.x, odometry.twist.twist.linear.y};
    double current_velocity = velocity_vector.len();

    SpeedPlan plan;

    double required_time =
        ros_time_to_seconds(it->header.stamp) - ros_time_to_seconds(odometry.header.stamp);
    if (required_time < 0) required_time = 0;
    double required_velocity = 0;
    if (it != path.rbegin()) {
        required_velocity =
            geom::dist(Vec2d(it->pose.position), Vec2d(prev(it)->pose.position)) /
            (ros_time_to_seconds(prev(it)->header.stamp) - ros_time_to_seconds(it->header.stamp));
    }
    required_velocity = std::min(required_velocity, model.max_velocity);

    if (it != path.rbegin()) {
        plan = getPlanWithTimePrior(dist, required_time, required_velocity, current_velocity, model);
    } else {
        plan = getPlanWithVelocityPrior(dist, required_time, required_velocity, current_velocity, model);
    }

    Command command;

    command.acceleration = plan.acceleration;
    command.velocity = plan.velocity;
    command.curvature = 1 / trajectory->getRadius();

    if (trajectory->getDirection() == geom::Arc::Direction::RIGHT) command.curvature *= -1;

    return ControllerResultData{command, visual_info};
}

};  // namespace pure_pursuit
