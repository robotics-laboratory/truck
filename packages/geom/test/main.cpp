#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/angle.h"
#include "geom/angle_vector.h"
#include "geom/arc.h"
#include "geom/distance.h"
#include "geom/common.h"
#include "geom/line.h"
#include "geom/polygon.h"
#include "geom/polyline.h"
#include "geom/uniform_stepper.h"
#include "geom/vector.h"

#include <sstream>

using namespace truck::geom;

TEST(Angle, constructor) {
    const auto a = Angle::fromRadians(0);
    const auto b = Angle::fromDegrees(90);
    const auto c = Angle::fromVector(-1, 0);

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a, PI0, eps);
    ASSERT_GEOM_EQUAL(b, PI_2, eps);
    ASSERT_GEOM_EQUAL(c, PI, eps);
}

TEST(Angle, coversion) {
    const auto a = PI_2;

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a.radians(), M_PI / 2, eps);
    ASSERT_GEOM_EQUAL(a.degrees(), 90.0, eps);
}

TEST(Angle, literals) {
    using namespace truck::geom::literals;

    const auto a = 0_rad;
    const auto b = 90_deg;

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a, PI0, eps);
    ASSERT_GEOM_EQUAL(b, PI_2, eps);
}

TEST(Angle, print) {
    std::stringstream ss;
    ss << Angle::fromDegrees(90);
    ASSERT_EQ(ss.str(), "90.00'");
}

TEST(Angle, normalization) {
    const auto a = Angle::fromDegrees(90);
    const auto b = Angle::fromDegrees(360 + 45);
    const auto c = Angle::fromDegrees(-2 * 360 - 45);

    constexpr double eps = 1e-9;

    ASSERT_GEOM_EQUAL(a._0_2PI(), PI_2, eps);
    ASSERT_GEOM_EQUAL(b._0_2PI(), PI_4, eps);
    ASSERT_GEOM_EQUAL(c._0_2PI(), 7 * PI_4, eps);

    ASSERT_GEOM_EQUAL(a._mPI_PI(), PI_2, eps);
    ASSERT_GEOM_EQUAL(b._mPI_PI(), PI_4, eps);
    ASSERT_GEOM_EQUAL(c._mPI_PI(), -PI_4, eps);
}

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

TEST(Line, make) {
    Line l(1, 1, -2);

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

        const Pose begin{.pos = {0, 0}, .dir = AngleVec2::fromVector(1, 0)};
        const Vec2 end = {0, eps};

        const auto result = tryBuildArc(begin, end);
        ASSERT_TRUE(std::holds_alternative<std::monostate>(result));
    }

    {
        /* begin.dir and vector (begin, end) are counterdirected  */

        const Pose begin{.pos = {0, 0}, .dir = AngleVec2::fromVector(-1, 0)};
        const Vec2 end = {1, 0};

        const auto result = tryBuildArc(begin, end);
        ASSERT_TRUE(std::holds_alternative<std::monostate>(result));
    }

    {
        /* begin.dir and vector (begin, end) are codirected  */

        const Pose begin{.pos = {0, 0}, .dir = AngleVec2::fromVector(1, 0)};
        const Vec2 end = {1, 0};

        const auto result = tryBuildArc(begin, end);
        ASSERT_TRUE(std::holds_alternative<Segment>(result));
        const auto& segment = std::get<Segment>(result);

        ASSERT_GEOM_EQUAL(segment.begin, begin.pos);
        ASSERT_GEOM_EQUAL(segment.end, end);
    }

    {
        /* positive arc more less than PI */

        const Pose begin{.pos = {0, -1}, .dir = AngleVec2::fromVector(1, 0)};
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

        const Pose from{.pos = {0, 1}, .dir = AngleVec2::fromVector(1, 0)};
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

        const Pose from{.pos = {0, -1}, .dir = AngleVec2::fromVector(1, 0)};
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

        const Pose from{.pos = {-1, 0}, .dir = AngleVec2::fromVector(0, 1)};
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

TEST(UniformStepper, stepping) {
    constexpr double eps = 1e-7;
    {
        const Polyline polyline = {Vec2(0, 0), Vec2(0, 1), Vec2(3, 1), Vec2(3, 2)};
        auto it = polyline.ubegin();
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(0, 0), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(0, 1)), eps);
        ++it;
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(0, 1), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(1, 0)), eps);
        ++it;
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(1, 1), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(1, 0)), eps);
        ++it;
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(2, 1), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(1, 0)), eps);
        ++it;
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(3, 1), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(0, 1)), eps);
        ++it;
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(3, 2), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(0, 1)), eps);
        ASSERT_TRUE(it == polyline.uend());
        ++it;
        ASSERT_TRUE(it == polyline.uend());
    }
    {
        const Polyline polyline = {Vec2(0, 0), Vec2(1, 1)};
        auto it = polyline.ubegin(0.5);
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(0, 0), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(1, 1)), eps);
        ++it;
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(sqrt(0.125), sqrt(0.125)), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(1, 1)), eps);
        ++it;
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(2 * sqrt(0.125), 2 * sqrt(0.125)), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(1, 1)), eps);
        ++it;
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(1, 1));
        ASSERT_TRUE(it == polyline.uend());
    }
    {
        const Polygon polygon = {Vec2(0, 0), Vec2(1, 0), Vec2(1, 1)};
        auto it = UniformStepper(&polygon, polygon.begin() + 1);
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(1, 0), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(0, 1)), eps);
        ++it;
        ASSERT_GEOM_EQUAL((*it).pos, Vec2(1, 1), eps);
        ASSERT_GEOM_EQUAL((*it).dir, AngleVec2::fromVector(Vec2(0, 1)), eps);
    }
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
