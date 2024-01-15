#pragma once

#include "geom/angle_vector.h"
#include "geom/segment.h"
#include "geom/vector.h"

#include <optional>

namespace truck::geom {

struct Ray {
    Ray(const Vec2& origin, const AngleVec2& dir) : origin(origin), dir(dir) {}

    Vec2 origin;
    AngleVec2 dir;
};

std::optional<Vec2> getIntersection(const Ray& ray,
    const Segment& segment, double precision) noexcept;

}  // namespace truck::geom
