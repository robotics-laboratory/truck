#pragma once

#include "common/math.h"
#include "common/exception.h"

#include "geom/pose.h"

#include <optional>
#include <vector>

namespace truck::trajectory_planner {

struct State {
    geom::Pose pose;
    double velocity;
};

using States = std::vector<State>;

geom::Poses FindMotion(
    const geom::Pose& from, const geom::Pose& to, size_t max_step, double eps = 1e-7);

double MotionLength(const geom::Poses& motion, double inf = 1e18);

double MotionTime(
    double motion_length, double form_velocity, double to_velocity, double eps = 1e-7,
    double inf = 1e18);

}  // namespace truck::trajectory_planner