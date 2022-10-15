#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/common.h"
#include "geom/vector.h"
#include "geom/line.h"
#include "geom/distance.h"
#include "geom/arc.h"

using namespace truck::geom;

TEST(Vector, addition) {
    const Vec2 a = {1, 2};
    const Vec2 b = {4, 8};
    
    const auto c = a + b;
    ASSERT_GEOM_EQUAL(c.x, a.x + b.x);
    ASSERT_GEOM_EQUAL(c.y, a.y + b.y);
    
    const auto d = a - b;
    ASSERT_GEOM_EQUAL(d.x, a.x - b.x);
    ASSERT_GEOM_EQUAL(d.y, a.y - b.y);
}

TEST(Vector, multiplication) {
    const Vec2 a = {42, 84};
    const double c = 42;

    const auto b = a * c;
    ASSERT_GEOM_EQUAL(b.x, a.x * c);
    ASSERT_GEOM_EQUAL(b.y, a.y * c);
    
    const auto d = c * a;
    ASSERT_GEOM_EQUAL(d.x, a.x * c);
    ASSERT_GEOM_EQUAL(d.y, a.y * c);
}

TEST(Vector, division) {
    const Vec2 a = {1, 2};
    const double c = 42;
    
    const auto b = a / c;
    ASSERT_GEOM_EQUAL(b.x, a.x / c);
    ASSERT_GEOM_EQUAL(b.y, a.y / c);
}

TEST(Vector, compare) {
    const Vec2 a = {1, 3};
    const Vec2 b = {1.001, 2.999};
    
    ASSERT_GEOM_NOT_EQUAL(a, b);
    ASSERT_GEOM_NOT_EQUAL(a, b, 1e-5);
    ASSERT_GEOM_EQUAL(a, b, 1e-2);
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

    ASSERT_GEOM_EQUAL(v.rotate(PI / 2), Vec2{-1, 1}, 1e-9);
    ASSERT_GEOM_EQUAL(v.rotate(-PI / 2), Vec2{1, -1}, 1e-9);
    ASSERT_GEOM_EQUAL(v.rotate(PI), -v, 1e-9);
    ASSERT_GEOM_EQUAL(v.rotate(PI * 4), v, 1e-9);
}

TEST(Vector, radians) {
    const Vec2 a = {1, 1};
    ASSERT_GEOM_EQUAL(a.angle(), PI / 4, 1e-9);

    const Vec2 b = {-10, 0};
    ASSERT_GEOM_EQUAL(b.angle(), PI, 1e-9);

    const Vec2 c = {0, -0.5};
    ASSERT_GEOM_EQUAL(c.angle(), -PI / 2, 1e-9);
}

TEST(Line, make) {
    Line l(1, 1, -2);

    auto l1 = Line::fromTwoPoints({0, 2}, {1, 1});
    auto l2 = Line::fromPointAndNormal({-1, 3}, {1, 1});
    auto l3 = Line::fromPointAndCollinear({4, -2}, {-1, 1});

    ASSERT_GEOM_EQUAL(l, l1);
    ASSERT_GEOM_EQUAL(l, l2);
    ASSERT_GEOM_EQUAL(l, l3);
}

TEST(Distanceance, point_point) {
    const Vec2 a = {1, 1};
    const Vec2 b = {0, 2};

    ASSERT_GEOM_EQUAL(distanceSq(a, b), 2.0, 1e-9);
    ASSERT_GEOM_EQUAL(distance(a, b), std::sqrt(2.0), 1e-9);
}

TEST(Distanceance, line_point) {
    const Line l = {1, 1, 0};
    const Vec2 a = {1, 1};
    const Vec2 b = {-1, -1};

    ASSERT_GEOM_EQUAL(denormalizedDistance(l, a), denormalizedDistance(a, l));
    ASSERT_GEOM_EQUAL(distance(l, a), distance(a, l));
    ASSERT_LT(denormalizedDistance(l, a) * denormalizedDistance(l, b), 0);
    ASSERT_GEOM_EQUAL(distance(a, l), std::sqrt(2.0), 1e-9);
    ASSERT_GEOM_EQUAL(distance(b, l), std::sqrt(2.0), 1e-9);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}