#pragma once

#include "geom/vector.h"
#include "geom/triangle.h"

#include <vector>

namespace truck::geom {

struct Polygon {
    Polygon() = default;

    Polygon(const std::vector<Vec2>& points) : points(points) {}

    std::vector<Triangle> triangles() const noexcept;

    std::vector<Vec2> points;
};

using Polygons = std::vector<Polygon>;

}  // namespace truck::geom