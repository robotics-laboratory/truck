#pragma once

#include <cmath>
#include "common/math.h"

namespace truck::geom {

inline bool equal(double a, double b, double eps = 0) noexcept { return std::abs(a - b) <= eps; }

}  // namespace truck::geom
