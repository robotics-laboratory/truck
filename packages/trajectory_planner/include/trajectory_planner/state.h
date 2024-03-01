#pragma once

#include "geom/pose.h"

#include <vector>

namespace truck::trajectory_planner {

struct State {
    geom::Pose pose;
    double velocity;
};

using Stetes = std::vector<State>;

}  // namespace truck::trajectory_planner