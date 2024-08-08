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

TEST(Distance, segment_point) {
    {
        const Segment s = {{0, 0}, {2, 0}};
        const Vec2 p = {1, 0.84};
        ASSERT_GEOM_EQUAL(distance(p, s), 0.84, 1e-9);
    }
    {
        const Segment s = {{0, 0}, {2, 0}};
        const Vec2 p = {3, 0};
        ASSERT_GEOM_EQUAL(distance(p, s), 1.0, 1e-9);
    }
    {
        const Segment s = {{0, 0}, {2, 0}};
        const Vec2 p = {3, 1};
        ASSERT_GEOM_EQUAL(distance(p, s), std::sqrt(2), 1e-9);
    }
    {
        const Segment s = {{0, 0}, {2, 0}};
        const Vec2 p = {-2, 0};
        ASSERT_GEOM_EQUAL(distance(p, s), 2.0, 1e-9);
    }
    {
        const Segment s = {{0, 0}, {2, 0}};
        const Vec2 p = {-2, 2};
        ASSERT_GEOM_EQUAL(distance(p, s), std::sqrt(8), 1e-9);
    }
    {
        const Segment s = {{0, 0}, {2, 0}};
        const Vec2 p = {-2, 2};
        ASSERT_GEOM_EQUAL(distanceSq(p, s), 8.0, 1e-9);
    }
}
