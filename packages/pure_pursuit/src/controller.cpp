#include "pure_pursuit/controller.h"

#include "geom/arc.h"
#include "geom/pose.h"
#include "geom/distance.h"
#include "geom/msg.h"
#include "geom/vector.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace truck::pure_pursuit {

std::string_view toString(ControllerError e) {
    switch (e) {
        case ControllerError::kUnreachablePath:
            return "unreachable path";
        case ControllerError::kImpossibleBuildArc:
            return "impossible build arc";
        default:
            return "unknown";
    }
}

double Controller::getRadius(double velocity) const {
    return params_.radius.clamp(std::max(0.0, velocity));
}

ControllerResult Controller::operator()(
        const nav_msgs::msg::Odometry& odometry,
        const nav_msgs::msg::Path& path) {
    const geom::Pose pose = geom::toPose(odometry.pose.pose);

    const geom::Vec2 velocity = {
        odometry.twist.twist.linear.x,
        odometry.twist.twist.linear.y
    };

    if (path.poses.empty()) {
        return ControllerResult(Command::stop());
    }

    const geom::Pose finish_pose = geom::toPose(path.poses.back().pose);
    if (geom::distance(finish_pose.pos, pose.pos) < params_.tolerance) {
        return ControllerResult(Command::stop());
    }

    const double radius = getRadius(velocity.len());
    const auto it = std::find_if(
        path.poses.begin(), path.poses.end(),
        [&](const auto& p) {
            return geom::distance(geom::toVec2(p), pose.pos) < radius;
        }
    );

    if (it == path.poses.end()) {
        return ControllerResult(ControllerError::kUnreachablePath);
    }

    auto goal_it = std::find_if(it, path.poses.end() - 1, [&](const auto& p) {
        return geom::distance(geom::toVec2(p), pose.pos) > radius;
    });

    const auto goal = geom::toVec2(*goal_it);
    const auto variant = geom::tryBuildArc(pose, goal);

    Command command;
    command.target = goal;

    if (std::holds_alternative<geom::Arc>(variant)) {
        const geom::Arc& arc = std::get<geom::Arc>(variant);
        command.velocity = params_.velocity;
        command.curvature = arc.curv();
    } else if (std::holds_alternative<geom::Segment>(variant)) {
        command.velocity = params_.velocity;
        command.curvature = 0.0;
    } else {
        return ControllerResult(ControllerError::kImpossibleBuildArc);
    }
        
    return ControllerResult{command};
}

}  // namespace truck::pure_pursuit
