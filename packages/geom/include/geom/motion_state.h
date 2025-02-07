#pragma once

#include "geom/pose.h"

namespace truck::geom {

struct MotionState {
    Vec2 pos;
    AngleVec2 dir;
    double curvature = 0;

    operator Pose() const { return Pose{.pos = pos, .dir = dir}; }
    MotionState& operator=(const MotionState& other) {
        pos = other.pos;
        dir = other.dir;
        curvature = other.curvature;
    }

    MotionState& operator=(const Pose& other) {
        pos = other.pos;
        dir = other.dir;
        curvature = 0;
    }
};
}  // namespace truck::geom
