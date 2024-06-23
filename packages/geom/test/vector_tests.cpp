#include <gtest/gtest.h>

#include "geom/angle_vector.h"
#include "geom/test/equal_assert.h"
#include "geom/vector.h"

using namespace truck::geom;

TEST(Vector, constructor) {
    const auto a = Vec2{1, 3};
    const auto b = Vec2::fromAngle(PI_2);
    const auto c = Vec2(AngleVec2(PI));

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a.x, 1., eps);
    ASSERT_GEOM_EQUAL(a.y, 3., eps);

    ASSERT_GEOM_EQUAL(b.x, 0., eps);
    ASSERT_GEOM_EQUAL(b.y, 1., eps);

    ASSERT_GEOM_EQUAL(c.x, -1., eps);
    ASSERT_GEOM_EQUAL(c.y, 0., eps);
}

TEST(Vector, len) {
    const Vec2 a = {1, 1};
    ASSERT_GEOM_EQUAL(a.len(), std::sqrt(2), 1e-9);

    const Vec2 b = {-1, -1};
    ASSERT_GEOM_EQUAL(b.len(), std::sqrt(2), 1e-9);

    const Vec2 c = {-1, 1};
    ASSERT_GEOM_EQUAL(c.len(), std::sqrt(2), 1e-9);
}

TEST(Vector, rotate) {
    const Vec2 v = {1, 1};

    const AngleVec2 a = PI_2;
    const AngleVec2 b = -PI_2;
    const AngleVec2 c = PI;
    const AngleVec2 d = PI * 4;

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a.apply(v), Vec2{-1, 1}, eps);
    ASSERT_GEOM_EQUAL(b.apply(v), Vec2{1, -1}, eps);
    ASSERT_GEOM_EQUAL(c.apply(v), -v, eps);
    ASSERT_GEOM_EQUAL(d.apply(v), v, eps);
}

TEST(Vector, to_angle_vector) {
    const Vec2 a = {1, 1};
    const Vec2 b = {-10, 0};
    const Vec2 c = {0, -0.5};

    ASSERT_GEOM_EQUAL(AngleVec2::fromVector(a), AngleVec2(PI_4), 1e-9);
    ASSERT_GEOM_EQUAL(AngleVec2::fromVector(b), AngleVec2(PI), 1e-9);
    ASSERT_GEOM_EQUAL(AngleVec2::fromVector(c), AngleVec2(-PI_2), 1e-9);
}
