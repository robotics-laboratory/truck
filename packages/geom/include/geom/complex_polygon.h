#pragma once

#include "geom/polygon.h"

#include <vector>

namespace truck::geom {

struct ComplexPolygon {
    ComplexPolygon() = default;

    ComplexPolygon(const Polygon& outer_poly) : outer_poly(outer_poly) {}

    ComplexPolygon(const Polygon& outer_poly, const Polygons& inner_polys)
        : outer_poly(outer_poly), inner_polys(inner_polys) {}

    std::vector<Triangle> triangles() const noexcept;

    Polygon outer_poly;
    Polygons inner_polys;
};

using ComplexPolygons = std::vector<ComplexPolygon>;

}  // namespace truck::geom