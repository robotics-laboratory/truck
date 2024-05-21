#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/arc.h"

using namespace truck::geom;

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
