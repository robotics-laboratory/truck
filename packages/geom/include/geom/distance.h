#pragma once

#include "geom/point.h"

#include <cmath>

namespace geom {

double DistanceSq(const Point2& a, const Point2& b) noexcept;

inline double Distance(const Point2& a, const Point2& b) { return std::sqrt(DistanceSq(a, b)); }

} // namespace geom