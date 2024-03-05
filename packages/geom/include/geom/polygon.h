#pragma once

#include "geom/vector.h"
#include "geom/triangle.h"
#include "geom/segment.h"

#include <vector>

namespace truck::geom {

enum class Orientation { COUNTERCLOCKWISE = 1, CLOCKWISE = -1 };

/** 2D Polygon
 *
 * The following assumptions are imposed on the geom::Polygon object:
 * 1. geom::Polygon is a simple polygon (that is, there are no self-intersections in it).
 * 2. the polygon is not degenerate (that is, it has at least 3 vertices).
 * 3. the vertices of geom::Polygon do not contain 3 consecutive vertices lying on the same line.
 * 4. the list of vertices does not require explicit closure (that is, for the ABC polygon, the list
 * of vertices has the form [A, B, C], and not [A, B, C, A]).
 */

struct Polygon : public std::vector<Vec2> {
    using vector::vector;
    using vector::operator=;

    size_t segmentNumber() const noexcept;

    Segment segment(size_t i) const noexcept;

    std::vector<Triangle> triangles() const noexcept;
    bool isConvex() const noexcept;
    Orientation orientation() const noexcept;
    Segments segments() const noexcept;
};

using Polygons = std::vector<Polygon>;

Polygon clip(
    const Polygon& boundary_polygon, const Polygon& clipped_polygon,
    const double eps = 1e-4) noexcept;

}  // namespace truck::geom