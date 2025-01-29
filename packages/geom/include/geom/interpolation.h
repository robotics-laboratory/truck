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
        .position = lerp(from.position, to.position, t),
        .yaw = lerp(from.yaw, to.yaw, t),
        .curvature = lerp(from.curvature, to.curvature, t),
    };
}

}  // namespace truck::geom
