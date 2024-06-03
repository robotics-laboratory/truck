#include <gtest/gtest.h>

#include "geom/intersection.h"
#include "geom/line.h"
#include "geom/test/equal_assert.h"

using namespace truck::geom;

TEST(Line, make) {
    Line const l(1, 1, -2);

    auto l1 = Line::fromTwoPoints({0, 2}, {1, 1});
    auto l2 = Line::fromPointAndNormal({-1, 3}, {1, 1});
    auto l3 = Line::fromPointAndCollinear({4, -2}, {-1, 1});

    ASSERT_GEOM_EQUAL(l, l1);
    ASSERT_GEOM_EQUAL(l, l2);
    ASSERT_GEOM_EQUAL(l, l3);
}

TEST(Line, intersect) {
    constexpr double eps = 1e-7;

    {
        auto l1 = Line::fromTwoPoints(Vec2(0, 0), Vec2(1, 1));
        auto l2 = Line::fromTwoPoints(Vec2(0, 1), Vec2(1, 0));

        ASSERT_GEOM_EQUAL(*intersect(l1, l2), Vec2(0.5, 0.5), eps);
    }

    {
        auto l1 = Line::fromTwoPoints(Vec2(0, 0), Vec2(0, 1));
        auto l2 = Line::fromTwoPoints(Vec2(1, 0), Vec2(1, 1));

        ASSERT_EQ(intersect(l1, l2), std::nullopt);
    }
}
