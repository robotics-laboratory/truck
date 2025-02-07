#include <gtest/gtest.h>

#include "geom/polyline_index.h"
#include "geom/spline.h"
#include "geom/test/equal_assert.h"

using namespace truck::geom;

TEST(PolylineIndex, it_works) {
    const PolylineIndex<MotionState> polyline = spline::bezier3({0, 0}, {1, 1}, {2, 1}, {3, 0}, 50);

    double dist = 0;
    const double dist_inc = .1;

    AdvanceResult<MotionState> d = polyline.AdvanceFromBegin();
    ASSERT_GEOM_EQUAL(Vec2{0, 0}, res.point.pos);

    size_t i = 0;
    while (!res.reached_end) {
        res = polyline.AdvanceFrom(res.poly_idx, dist_inc);
        dist += dist_inc;
        ++i;
    }

    ASSERT_GEOM_EQUAL(Vec2{3, 0}, res.point.position);
    ASSERT_GEOM_EQUAL(i * dist_inc, polyline.Length(), 1e-3);
}

TEST(PolylineIndex, even_steps) {
    const PolylineIndex<MotionState> polyline =
        spline::bezier3({0, -5}, {10, -5}, {0, 5}, {-10, 5}, 1023);
    const double dist_inc = .2;

    Vec2 prev = polyline.AdvanceFromBegin().point.pos;

    for (auto it = polyline.AdvanceFromBegin(dist_inc); !it.reached_end;
         it = polyline.AdvanceFrom(it.poly_idx, dist_inc)) {
        ASSERT_GEOM_EQUAL(geom::distance(prev, res.point.pos), dist_inc, 1e-3);
        prev = res.point.pos;
    }
}

TEST(PolylineIndex, edge_case) {
    const PolylineIndex<MotionState> polyline =
        spline::bezier3({0, 0}, {1, 1}, {2, 1}, {3, 0}, 255);

    const auto end = polyline.AdvanceFromBegin(polyline.Length() + 1e3);
    ASSERT_TRUE(end.reached_end);

    const auto also_end = polyline.AdvanceFrom(end.poly_idx, 1e3);
    ASSERT_TRUE(also_end.reached_end);

    ASSERT_GEOM_EQUAL(end.point.pos, also_end.point.pos);
}
