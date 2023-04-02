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

namespace {

constexpr size_t kNoIdx = -1;

size_t getEgoSegmentIndex(
        const motion::States& states,
        const geom::Pose& ego_pose,
        double max_distance) {
    double min_distnace_sq = squared(max_distance);
    size_t ego_segment_idx = kNoIdx;

    for (size_t end = 1; end < states.size(); ++end) {
        const size_t begin = end - 1;

        const geom::Segment segment = {states[begin].pose.pos, states[end].pose.pos};

        const double distance_sq = geom::distanceSq(ego_pose.pos, segment);
        // get closest segment
        if (distance_sq >= min_distnace_sq) {
            if (ego_segment_idx == kNoIdx) {
                continue;
            }

            break;
        }

        min_distnace_sq = distance_sq;
        ego_segment_idx = begin;
    }

    return ego_segment_idx;
}

} // namespace 

std::string_view toString(Error e) {
    switch (e) {
        case Error::kUnreachablePath:
            return "unreachable path";
        case Error::kImpossibleBuildArc:
            return "impossible build arc";
        default:
            return "unknown";
    }
}

double PurePursuit::getRadius(double velocity) const {
     return params_.radius.clamp(std::max(0.0, velocity));
}

Result PurePursuit::operator()(
        const geom::Localization& localization,
        const motion::Trajectory& trajectory) {
    const auto& states = trajectory.states;
    if (states.empty()) {
        return Result(Command::stop());
    }

    const size_t index = getEgoSegmentIndex(states, localization.pose, params_.max_distance);
    if (index == kNoIdx) {
        return Result(Error::kUnreachablePath);
    }

    const double velocity = [&] {
        for (size_t i = index; i < states.size(); ++i) {
            if ((states[i].time - states[index].time) > params_.period.count()) {
                std::cerr << "i=" << i << states[i].velocity << std::endl;
                return states[i].velocity;
            }
        }

        std::cerr << "back=" << states.back().velocity << std::endl;
        return states.back().velocity;
    }();

    const geom::Pose& finish = states.back().pose;
    if (geom::distance(finish.pos, localization.pose.pos) < params_.tolerance) {
        return Result(Command::stop());
    }

    const double radius = getRadius(localization.velocity);
    const auto it = std::find_if(states.begin(), states.end(),
        [&](const auto& s) {
            return geom::distance(s.pose.pos, localization.pose.pos) < radius;
        }
    );

    if (it == states.end()) {
        return Result(Error::kUnreachablePath);
    }

    auto goal_it = std::find_if(
        it, states.end() - 1,
        [&](const auto& s) {
            return geom::distance(s.pose.pos, localization.pose.pos) > radius;
        }
    );

    const auto goal = goal_it->pose.pos;
    const auto variant = geom::tryBuildArc(localization.pose, goal);

    Command command;
    command.velocity = velocity;
    command.target = goal;

    if (std::holds_alternative<geom::Arc>(variant)) {
        const geom::Arc& arc = std::get<geom::Arc>(variant);
        command.curvature = arc.curv();
    } else if (std::holds_alternative<geom::Segment>(variant)) {
        command.curvature = 0.0;
    } else {
        return Result(Error::kImpossibleBuildArc);
    }

    return Result{command};
}

}  // namespace truck::pure_pursuit
