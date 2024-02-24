#pragma once

#include "geom/segment.h"
#include "geom/ray.h"
#include "geom/vector.h"

#include <optional>

namespace truck::geom {

std::optional<Vec2> intersect(const Ray& ray,
    const Segment& segment, double precision) noexcept;

}  // namespace truck::geom
