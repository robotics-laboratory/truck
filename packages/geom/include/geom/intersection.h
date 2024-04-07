#pragma once

#include "geom/ray.h"
#include "geom/line.h"
#include "geom/segment.h"
#include "geom/polygon.h"
#include "geom/vector.h"

#include <optional>

namespace truck::geom {

bool intersect(
    const Segment& seg1, const Segment& seg2, const double eps = 1e-4) noexcept;

bool intersect(const Polygon& polygon, const Segment& seg, const double eps = 1e-4) noexcept;

std::optional<Vec2> intersect(const Line& l1, const Line& l2, const double eps = 1e-4) noexcept;

std::optional<Vec2> intersect(const Ray& ray, const Segment& segment, double precision) noexcept;

}  // namespace truck::geom
