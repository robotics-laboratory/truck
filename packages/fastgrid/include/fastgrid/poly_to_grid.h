#pragma once

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"

#include "geom/complex_polygon.h"
#include "geom/polygon.h"

namespace truck::fastgrid {

void PolyToGrid(const geom::Polygon& poly, U8Grid& grid);

U8GridHolder PolyToGrid(
    const geom::Polygon& poly, const Size& size, double resolution,
    const std::optional<geom::Pose>& origin);

void PolyToGrid(const geom::ComplexPolygon& poly, U8Grid& grid);

U8GridHolder PolyToGrid(
    const geom::ComplexPolygon& poly, const Size& size, double resolution,
    const std::optional<geom::Pose>& origin);

}  // namespace truck::fastgrid