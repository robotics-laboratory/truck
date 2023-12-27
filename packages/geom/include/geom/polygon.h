#pragma once

#include "geom/vector.h"
#include "geom/triangle.h"

#include <vector>

namespace truck::geom {

struct Polygon : public std::vector<Vec2> {
    using vector::vector;

    std::vector<Triangle> triangles() const noexcept;
};

using Polygons = std::vector<Polygon>;

}  // namespace truck::geom