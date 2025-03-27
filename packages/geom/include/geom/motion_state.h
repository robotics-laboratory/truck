#pragma once

#include "geom/pose.h"
#include "geom/common.h"
#include "geom/polyline.h"
#include <vector>

namespace truck::geom {

struct MotionState {
    Vec2 pos;
    AngleVec2 dir;
    double curvature = 0;

    Pose pose() const { return Pose{.pos = pos, .dir = dir}; }
};

using MotionStates = std::vector<MotionState>;

Poses toPoses(const MotionStates& states) noexcept;
Polyline toPolyline(const MotionStates& states) noexcept;

struct MotionStateLinearInterpolator {
    MotionState operator()(const MotionState& from, const MotionState& to, double t) const {
        return {
            .pos = interpolate(from.pos, to.pos, t),
            .dir = interpolate(from.dir, to.dir, t),
            .curvature = interpolate(from.curvature, to.curvature, t),
        };
    }

    double operator()(double from, const double to, double t) const {
        return interpolate(from, to, t);
    }
};

}  // namespace truck::geom
