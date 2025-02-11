#pragma once

#include <cmath>
#include "common/math.h"

namespace truck::geom {

inline bool equal(double a, double b, double eps = 0) noexcept { return std::abs(a - b) <= eps; }
inline double interpolate(double a, double b, double t) noexcept {
    VERIFY(0 <= t && t <= 1);
    return (1 - t) * a + t * b;
}
}  // namespace truck::geom
