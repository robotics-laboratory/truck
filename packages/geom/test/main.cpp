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

TEST(Arc, try_build_arc) {
    constexpr double eps = 1e-6;

    {
        /* begin and end are to close */

        const Pose begin{.pos = {0, 0}, .dir = {1, 0}};
        const Vec2 end = {0, eps};

        const auto result = tryBuildArc(begin, end);
        ASSERT_TRUE(std::holds_alternative<std::monostate>(result));
    }

    {
        /* begin.dir and vector (begin, end) are counterdirected  */

        const Pose begin{.pos = {0, 0}, .dir = {-1, 0}};
        const Vec2 end = {1, 0};

        const auto result = tryBuildArc(begin, end);
        ASSERT_TRUE(std::holds_alternative<std::monostate>(result));
    }

    {
        /* begin.dir and vector (begin, end) are codirected  */

        const Pose begin{.pos = {0, 0}, .dir = {1, 0}};
        const Vec2 end = {1, 0};

        const auto result = tryBuildArc(begin, end);
        ASSERT_TRUE(std::holds_alternative<Segment>(result));
        const auto& segment = std::get<Segment>(result);

        ASSERT_GEOM_EQUAL(segment.begin, begin.pos);
        ASSERT_GEOM_EQUAL(segment.end, end);
    }

    {
        /* positive arc more less than PI */

        const Pose begin{.pos = {0, -1}, .dir = {1, 0}};
        const Vec2 end = {1, 0};

        const auto result = tryBuildArc(begin, end);

        ASSERT_TRUE(std::holds_alternative<Arc>(result));
        const auto& arc = std::get<Arc>(result);

        ASSERT_GEOM_EQUAL(arc.center, {0, 0}, eps);
        ASSERT_GEOM_EQUAL(arc.radius, 1.0, eps);
        ASSERT_GEOM_EQUAL(arc.begin, {0, -1}, eps);
        ASSERT_GEOM_EQUAL(arc.delta, PI_2, eps);
    }

    {
        /* negative arc less than PI */

        const Pose from{.pos = {0, 1}, .dir = {1, 0}};
        const Vec2 to = {1, 0};

        const auto result = tryBuildArc(from, to);
        ASSERT_TRUE(std::holds_alternative<Arc>(result));
        const auto& arc = std::get<Arc>(result);

        ASSERT_GEOM_EQUAL(arc.center, {0, 0}, eps);
        ASSERT_GEOM_EQUAL(arc.radius, 1.0, eps);
        ASSERT_GEOM_EQUAL(arc.begin, {0, 1}, eps);
        ASSERT_GEOM_EQUAL(arc.delta, -PI_2, eps);
    }

    {
        /* positive arc more than PI */

        const Pose from{.pos = {0, -1}, .dir = {1, 0}};
        const Vec2 to = {-1, 0};

        const auto result = tryBuildArc(from, to);
        ASSERT_TRUE(std::holds_alternative<Arc>(result));
        const auto& arc = std::get<Arc>(result);

        ASSERT_GEOM_EQUAL(arc.center, {0, 0}, eps);
        ASSERT_GEOM_EQUAL(arc.radius, 1.0, eps);
        ASSERT_GEOM_EQUAL(arc.begin, {0, -1}, eps);
        ASSERT_GEOM_EQUAL(arc.delta, 3 * PI_2, eps);
    }

    {
        /* negative arc more than PI */

        const Pose from{.pos = {-1, 0}, .dir = {0, 1}};
        const Vec2 to = {0, -1};

        const auto result = tryBuildArc(from, to);
        ASSERT_TRUE(std::holds_alternative<Arc>(result));
        const auto& arc = std::get<Arc>(result);

        ASSERT_GEOM_EQUAL(arc.center, {0, 0}, eps);
        ASSERT_GEOM_EQUAL(arc.radius, 1.0, eps);
        ASSERT_GEOM_EQUAL(arc.begin, {-1, 0}, eps);
        ASSERT_GEOM_EQUAL(arc.delta, -3 * PI_2, eps);
    }
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}