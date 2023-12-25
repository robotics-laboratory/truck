#pragma once

#include "geom/polygon.h"

#include <vector>

namespace truck::geom {

struct ComplexPolygon {
    ComplexPolygon() = default;

    ComplexPolygon(const Polygon& outer) : outer(outer) {}

    ComplexPolygon(const Polygon& outer, const Polygons& inners) : outer(outer), inners(inners) {}

    std::vector<Triangle> triangles() const noexcept;

    Polygon outer;
    Polygons inners;
};

using ComplexPolygons = std::vector<ComplexPolygon>;

}  // namespace truck::geom