#pragma once

#include "geom/pose.h"
#include "geom/interpolation.h"

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

MotionState interpolate(const MotionState& from, const MotionState& to, double t) {
    return {
        .pos = interpolate(from.pos, to.pos, t),
        .dir = interpolate(from.dir, to.dir, t),
        .curvature = interpolate(from.curvature, to.curvature, t),
    };
}

}  // namespace truck::geom
