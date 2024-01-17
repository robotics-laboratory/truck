#pragma once

#include "geom/angle_vector.h"
#include "geom/segment.h"
#include "geom/vector.h"

namespace truck::geom {

struct Ray {
    Ray(const Vec2& origin, const AngleVec2& dir) : origin(origin), dir(dir) {}

    Vec2 origin;
    AngleVec2 dir;
};

bool checkIntersection(const Ray& ray, const Segment& segment,
    Vec2& intersection, double precision) noexcept;

}  // namespace truck::geom
