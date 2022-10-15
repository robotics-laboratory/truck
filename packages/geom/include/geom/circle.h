#pragma once

#include "geom/vector.h"

#include <ostream>

namespace truck::geom {

struct Circle {
    Vec2 center;
    double radius;
};

bool near(const Circle& a, const Circle& b, double eps = 0) noexcept;

std::ostream& operator<<(std::ostream& out, const Circle& c) noexcept;

}  // namespace truck::geom
