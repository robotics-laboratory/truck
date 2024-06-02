#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/angle_vector.h"

using namespace truck::geom;

TEST(AngleVec2, constructor) {
    const auto a = AngleVec2::fromVector(1, 1);
    const auto b = AngleVec2(PI_6);

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a.angle(), PI_4, eps);
    ASSERT_GEOM_EQUAL(a.x(), std::sqrt(2) / 2, eps);
    ASSERT_GEOM_EQUAL(a.y(), std::sqrt(2) / 2, eps);

    ASSERT_GEOM_EQUAL(b.angle(), PI_6, eps);
    ASSERT_GEOM_EQUAL(b.x(), std::sqrt(3) / 2, eps);
    ASSERT_GEOM_EQUAL(b.y(), 0.5, eps);
}

TEST(AngleVec2, operation) {
    const auto a = AngleVec2(PI_2);
    const auto b = AngleVec2(PI_2);

    const auto sum = a + b;
    const auto diff = a - b;
    const auto inv = a.inv();

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(sum.angle(), PI, eps);
    ASSERT_GEOM_EQUAL(sum.x(), -1., eps);
    ASSERT_GEOM_EQUAL(sum.y(), 0., eps);

    ASSERT_GEOM_EQUAL(diff.angle(), PI0, eps);
    ASSERT_GEOM_EQUAL(diff.x(), 1., eps);
    ASSERT_GEOM_EQUAL(diff.y(), 0., eps);

    ASSERT_GEOM_EQUAL(inv.angle(), -PI_2, eps);
    ASSERT_GEOM_EQUAL(inv.x(), 0., eps);
    ASSERT_GEOM_EQUAL(inv.y(), -1., eps);
}

TEST(AngleVec2, rotate) {
    const auto a = AngleVec2(Angle::zero());

    const auto left = a.left();
    const auto right = a.right();
    const auto inv = AngleVec2(-PI).apply(a);

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(left.angle(), PI_2, eps);
    ASSERT_GEOM_EQUAL(left.x(), 0., eps);
    ASSERT_GEOM_EQUAL(left.y(), 1., eps);

    ASSERT_GEOM_EQUAL(right.angle(), -PI_2, eps);
    ASSERT_GEOM_EQUAL(right.x(), 0., eps);
    ASSERT_GEOM_EQUAL(right.y(), -1., eps);

    ASSERT_GEOM_EQUAL(inv.angle(), -PI, eps);
    ASSERT_GEOM_EQUAL(inv.x(), -1., eps);
    ASSERT_GEOM_EQUAL(inv.y(), 0., eps);
}
