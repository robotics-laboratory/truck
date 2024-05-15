#pragma once

#include "geom/vector.h"

namespace truck::geom {

struct Triangle {
    Triangle(const Vec2& p1, const Vec2& p2, const Vec2& p3) : p1(p1), p2(p2), p3(p3) {}

    Vec2 p1;
    Vec2 p2;
    Vec2 p3;
};

}  // namespace truck::geom
