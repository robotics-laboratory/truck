#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/polyline.h"
#include "geom/uniform_stepper.h"

using namespace truck::geom;

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
