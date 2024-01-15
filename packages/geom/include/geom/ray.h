#pragma once

#include "geom/vector.h"
#include "geom/angle_vector.h"

namespace truck::geom {

struct Ray {
    Ray(const Vec2& origin, const AngleVec2& dir) : origin(origin), dir(dir) {}

    Vec2 origin;
    AngleVec2 dir;
};

using Rays = std::vector<Ray>;

}  // namespace truck::geom