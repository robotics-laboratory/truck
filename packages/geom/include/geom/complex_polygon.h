#pragma once

#include "geom/polygon.h"

#include "common/exception.h"

#include <vector>

namespace truck::geom {

struct ComplexPolygon {
    ComplexPolygon() = default;

    ComplexPolygon(const Polygon& outer_poly, const Polygons& inner_polys)
        : outer_poly(outer_poly), inner_polys(inner_polys) {}

    ComplexPolygon(const std::vector<std::vector<std::pair<double, double>>>& polys_list) {
        // need to have at least one outer poly a.k.a. polygon hole
        VERIFY(polys_list.size() > 1);
        
        for (size_t i = 0; i < polys_list.size(); i++) {
            // the first polygon in given list is an outer polygon
            if (i == 0) {
                outer_poly = geom::Polygon(polys_list[i]);
            } else {
                inner_polys.push_back(geom::Polygon(polys_list[i]));
            }
        }
    }

    Polygon outer_poly;
    Polygons inner_polys;
};

using ComplexPolygons = std::vector<ComplexPolygon>;

}  // namespace truck::geom