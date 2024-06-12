#include "pure_pursuit/pure_pursuit.h"

#include "geom/arc.h"
#include "geom/distance.h"
#include "geom/msg.h"
#include "geom/pose.h"
#include "geom/segment.h"
#include "geom/vector.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace truck::pure_pursuit {

std::string_view toString(Error e) {
    switch (e) {
        case Error::kUnknown:
            return "unknown problem";
        case Error::kNoProjection:
            return "no projection";
        case Error::kUnreachableProjection:
            return "unreachable projection";
        case Error::kImpossibleBuildArc:
            return "impossible build arc";
        default:
            throw Exception() << "Unknown error code: " << static_cast<uint8_t>(e);
    }
}

double PurePursuit::getRadius(double velocity) const {
    return params_.radius.clamp(std::max(0.0, velocity));
}

Result PurePursuit::operator()(
    const geom::Localization& localization, const motion::Trajectory& trajectory) {
    const auto& states = trajectory.states;
    if (states.empty()) {
        return {Command::stop()};
    }

    const auto ego_state = trajectory.byProjection(localization.pose, params_.max_distance);
    if (!ego_state) {
        return {Error::kNoProjection};
    }

    if (!ego_state->reachable()) {
        return {Error::kUnreachableProjection};
    }

    const double radius = getRadius(localization.velocity);
    const auto goal_state = trajectory.byDistance(ego_state->distance + radius);

    const auto goal = goal_state.pose.pos;
    const auto variant = geom::tryBuildArc(localization.pose, goal);
    const auto scheduled = trajectory.byTime(ego_state->getTime() + params_.period.count());

    Command command;
    command.velocity = scheduled.velocity;
    command.target = goal;

    if (std::holds_alternative<geom::Arc>(variant)) {
        const geom::Arc& arc = std::get<geom::Arc>(variant);
        command.curvature = arc.curv();
    } else if (std::holds_alternative<geom::Segment>(variant)) {
        command.curvature = 0.0;
    } else {
        return {Error::kImpossibleBuildArc};
    }

    return Result{command};
}

}  // namespace truck::pure_pursuit
