#pragma once

#include "geom/line.h"
#include "geom/segment.h"
#include "geom/polygon.h"

#include <optional>

namespace truck::geom {

bool intersect(
    const geom::Segment& seg1, const geom::Segment& seg2, const double eps = 1e-4) noexcept;

bool intersect(const geom::Polygon& polygon, const Segment& seg, const double eps = 1e-4) noexcept;

std::optional<Vec2> intersect(const Line& l1, const Line& l2, const double eps = 1e-4) noexcept;

}  // namespace truck::geom