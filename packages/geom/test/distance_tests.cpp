#include <gtest/gtest.h>

#include "geom/distance.h"
#include "geom/test/equal_assert.h"

using namespace truck::geom;

TEST(Distance, point_point) {
    const Vec2 a = {1, 1};
    const Vec2 b = {0, 2};

    ASSERT_GEOM_EQUAL(distanceSq(a, b), 2.0, 1e-9);
    ASSERT_GEOM_EQUAL(distance(a, b), std::sqrt(2.0), 1e-9);
}

TEST(Distance, line_point) {
    const Line l = {1, 1, 0};
    const Vec2 a = {1, 1};
    const Vec2 b = {-1, -1};

    ASSERT_GEOM_EQUAL(denormalizedDistance(l, a), denormalizedDistance(a, l));
    ASSERT_GEOM_EQUAL(distance(l, a), distance(a, l));
    ASSERT_LT(denormalizedDistance(l, a) * denormalizedDistance(l, b), 0);
    ASSERT_GEOM_EQUAL(distance(a, l), std::sqrt(2.0), 1e-9);
    ASSERT_GEOM_EQUAL(distance(b, l), std::sqrt(2.0), 1e-9);
}
