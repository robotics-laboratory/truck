#include <gtest/gtest.h>

#include "geom/polyline_index.h"
#include "geom/spline.h"
#include "geom/test/equal_assert.h"

using namespace truck::geom;

TEST(PolylineIndex, it_works) {
    const PolylineIndex<MotionState> polyline =
        spline::bezier3({0, 0}, {1, 1}, {2, 1}, {3, 0}, 50);

    double dist = 0;
    const double dist_inc = .1;

    AdvanceResult<MotionState> res = polyline.AdvanceFromBegin(0);
    ASSERT_GEOM_EQUAL(Vec2{0, 0}, res.point.position);

    size_t i = 0;
    while (!res.reached_end) {
        res = polyline.AdvanceFrom(res.poly_idx, dist_inc);
        dist += dist_inc;
        ++i;
    }

    ASSERT_GEOM_EQUAL(Vec2{3, 0}, res.point.position);
}