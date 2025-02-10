#pragma once
#include "geom/motion_state.h"

namespace truck::geom {

template<typename T>
struct LinearInterpolator {
    T operator()(const T& from, const T& to, double t) const { return interpolate(from, to, t); }
};

double interpolate(double a, double b, double t) { return a * (1 - t) + b * t; }

}  // namespace truck::geom
