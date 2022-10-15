#pragma once

#include "geom/common.h"
#include "geom/vector.h"
#include "geom/polyline.h"

namespace truck::geom {

struct Arc {
    Vec2 start;
    Vec2 end;
    double curvature;

    static Arc byTwoPointsAndTangent(const Vec2& start, const Vec2& end, const Vec2& tangent) noexcept;
};

}  // namespace truck::geom
