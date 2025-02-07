#pragma once
#include "geom/motion_state.h"

namespace truck::geom {
template<typename T>
struct LinearInterpolator {
    T operator()(const T& from, const T& to, double t) const { return lerp<T>(from, to, t); }
};

template<>
MotionState lerp<MotionState>(const MotionState& from, const MotionState& to, double t) {
    return {
        .pos = lerp(from.pos, to.pos, t),
        .dir = lerp(from.dir, to.dir, t),
        .curvature = lerp(from.curvature, to.curvature, t),
    };
}

}  // namespace truck::geom
