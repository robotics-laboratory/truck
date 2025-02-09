#pragma once

#include "geom/pose.h"

namespace truck::geom {

struct MotionState {
    Vec2 position;
    Angle yaw;
    double curvature = 0;

    Pose pose() const { return Pose{.pos = position, .dir = yaw}; }
};
}  // namespace truck::geom
