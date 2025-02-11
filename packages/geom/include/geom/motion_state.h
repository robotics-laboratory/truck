#pragma once

#include "geom/pose.h"

namespace truck::geom {

struct MotionState {
    Vec2 pos;
    AngleVec2 dir;
    double curvature = 0;

    MotionState& operator=(const MotionState& other) {
        pos = other.pos;
        dir = other.dir;
        curvature = other.curvature;
        return *this;
    }
};

struct MotionStateLinearInterpolator {
    MotionState operator()(const MotionState& from, const MotionState& to, double t) const {
        return {
            .pos = interpolate(from.pos, to.pos, t),
            .dir = interpolate(from.dir, to.dir, t),
            .curvature = (1 - t) * from.curvature + t * to.curvature,
        };
    }
};

// MotionState interpolate(const MotionState& from, const MotionState& to, double t) {
//     return {
//         .pos = interpolate(from.pos, to.pos, t),
//         .dir = interpolate(from.dir, to.dir, t),
//         .curvature = (1 - t) * from.curvature + t * to.curvature,
//     };
// }

}  // namespace truck::geom
