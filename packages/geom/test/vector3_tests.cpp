#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/vector3.h"

using namespace truck::geom;

TEST(Vector3, constructor) {
    const auto a = Vec3();
    const auto b = Vec3{2, 5, 3};
    const auto vec2 = Vec2{1, 3};
    const auto c = Vec3(vec2, 4);
    const auto d = Vec3(vec2);

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a.x, 0., eps);
    ASSERT_GEOM_EQUAL(a.y, 0., eps);
    ASSERT_GEOM_EQUAL(a.z, 0., eps);

    ASSERT_GEOM_EQUAL(b.x, 2., eps);
    ASSERT_GEOM_EQUAL(b.y, 5., eps);
    ASSERT_GEOM_EQUAL(b.z, 3., eps);

    ASSERT_GEOM_EQUAL(c.x, 1., eps);
    ASSERT_GEOM_EQUAL(c.y, 3., eps);
    ASSERT_GEOM_EQUAL(c.z, 4., eps);

    ASSERT_GEOM_EQUAL(d.x, 1., eps);
    ASSERT_GEOM_EQUAL(d.y, 3., eps);
    ASSERT_GEOM_EQUAL(d.z, 0., eps);
}

TEST(Vector3, len) {
    const Vec3 a = {1, 1, 1};
    const Vec3 b = {-1, -1, -1};
    const Vec3 c = {-1, 1, 0};
    const Vec3 d = {2, 5, 7};

    ASSERT_GEOM_EQUAL(a.len(), std::sqrt(3), 1e-9);
    ASSERT_GEOM_EQUAL(b.len(), std::sqrt(3), 1e-9);
    ASSERT_GEOM_EQUAL(c.len(), std::sqrt(2), 1e-9);
    ASSERT_GEOM_EQUAL(d.len(), std::sqrt(78), 1e-9);
}
