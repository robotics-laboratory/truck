#pragma once

#include <cmath>
#include "common/math.h"

namespace truck::geom {

inline bool equal(double a, double b, double eps = 0) noexcept { return std::abs(a - b) <= eps; }

template<typename T>
T lerp(const T& from, const T& to, double t) {
    interpolate(from, to, t);
}

template<VectorSpace V>
V lerp(const V& from, const V& to, double t) {
    return from * (1 - t) + to * t;
}

// TODO: "vectorized" lerp for tuples/ties

}  // namespace truck::geom
