#pragma once

#include "geom/polygon.h"

#include <vector>

namespace truck::geom {

struct ComplexPolygon {
    ComplexPolygon() = default;

    ComplexPolygon(Polygon outer) : outer(std::move(outer)) {}

    ComplexPolygon(Polygon outer, Polygons inners)
        : outer(std::move(outer)), inners(std::move(inners)) {}

    std::vector<Triangle> triangles() const noexcept;
    Segments segments() const noexcept;

    Polygon outer;
    Polygons inners;
};

using ComplexPolygons = std::vector<ComplexPolygon>;

}  // namespace truck::geom