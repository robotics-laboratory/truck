#include "fastgrid/poly_to_grid.h"

#include "common/math.h"

namespace truck::fastgrid {

namespace impl {

__always_inline void DrivingByX(
    const geom::Vec2& rel_p1, const geom::Vec2& rel_p2, fastgrid::U8Grid& grid) noexcept {
    double k = (rel_p2.y - rel_p1.y) / (rel_p2.x - rel_p1.x);
    double y = rel_p1.y / grid.resolution;
    for (int64_t x = grid.getIndex(rel_p1).x; x <= grid.getIndex(rel_p2).x; ++x, y += k) {
        grid.data[static_cast<int64_t>(y) * grid.size.width + x] = 1;
    }
}

__always_inline void DrivingByY(
    const geom::Vec2& rel_p1, const geom::Vec2& rel_p2, fastgrid::U8Grid& grid) noexcept {
    double k = (rel_p2.x - rel_p1.x) / (rel_p2.y - rel_p1.y);
    double x = rel_p1.x / grid.resolution;
    for (int64_t y = grid.toIndex(rel_p1).y; y <= grid.toIndex(rel_p2).y; ++y, x += k) {
        grid.data[y * grid.size.width + static_cast<int64_t>(x)] = 1;
    }
}

__always_inline void SegmentToGrid(
    const geom::Vec2& p1, const geom::Vec2& p2, fastgrid::U8Grid& grid) noexcept {
    auto rel_p1 = grid.transform(p1);
    auto rel_p2 = grid.transform(p2);
    if (abs(rel_p2.x - rel_p1.x) >= abs(rel_p2.y - rel_p1.y)) {
        if (rel_p1.x <= rel_p2.x) {
            DrivingByX(rel_p1, rel_p2, grid);
        } else {
            DrivingByX(rel_p2, rel_p1, grid);
        }
    } else {
        if (rel_p1.y <= rel_p2.y) {
            DrivingByY(rel_p1, rel_p2, grid);
        } else {
            DrivingByY(rel_p2, rel_p1, grid);
        }
    }
}

__always_inline void PolyToGrid(const geom::Polygon& poly, fastgrid::U8Grid& grid) noexcept {
    for (auto it = poly.begin(); it + 1 != poly.end(); ++it) {
        SegmentToGrid(*it, *(it + 1), grid);
    }
    SegmentToGrid(poly.back(), poly.front(), grid);
}

__always_inline void PolyToGrid(const geom::ComplexPolygon& poly, U8Grid& grid) noexcept {
    impl::PolyToGrid(poly.outer, grid);
    for (const auto& inner : poly.inners) {
        impl::PolyToGrid(inner, grid);
    }
}

}  // namespace impl

void PolyToGrid(const geom::Polygon& poly, U8Grid& grid) { impl::PolyToGrid(poly, grid); }

U8GridHolder PolyToGrid(
    const geom::Polygon& poly, const Size& size, double resolution,
    const std::optional<geom::Pose>& origin) {
    auto result = makeGrid<uint8_t>(size, resolution, origin);
    PolyToGrid(poly, *result);
    return result;
}

void PolyToGrid(const geom::ComplexPolygon& poly, U8Grid& grid) { impl::PolyToGrid(poly, grid); }

U8GridHolder PolyToGrid(
    const geom::ComplexPolygon& poly, const Size& size, double resolution,
    const std::optional<geom::Pose>& origin) {
    auto result = makeGrid<uint8_t>(size, resolution, origin);
    PolyToGrid(poly, *result);
    return result;
}

}  // namespace truck::fastgrid
