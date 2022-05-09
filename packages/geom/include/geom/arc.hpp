#pragma once

#include <cmath>

#include "geom/circle.hpp"
#include "geom/common.hpp"

namespace geom {

struct Arc {
    Circled circle;
    double begin_radians;
    double end_radians;
    bool direction;

    double get_length() { return circle.r * std::abs(end_radians - begin_radians); }
};

[[gnu::always_inline, nodiscard, gnu::pure]] inline bool eq(const Arc& a, const Arc& b,
                                                            double eps = 0) noexcept {
    return eq(a.circle, b.circle, eps) && eq(a.begin_radians, b.begin_radians, eps) &&
           eq(a.end_radians, b.end_radians, eps) && a.direction == b.direction;
}

}  // namespace geom
