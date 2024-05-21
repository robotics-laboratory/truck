#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/intersection.h"
#include "geom/polygon.h"

using namespace truck::geom;

TEST(Polygon, isComplex) {
    {
        const Polygon poly{Vec2(0, 0), Vec2(1, 0), Vec2(1, 1)};
        ASSERT_TRUE(poly.isConvex());
    }
    {
        const Polygon poly{Vec2(0, 0), Vec2(4, 0), Vec2(4, 3), Vec2(2, 2), Vec2(0, 3)};
        ASSERT_FALSE(poly.isConvex());
    }
}

TEST(Polygon, orientation) {
    {
        const Polygon poly{Vec2(0, 0), Vec2(1, 0), Vec2(1, 1), Vec2(0, 1)};
        ASSERT_EQ(poly.orientation(), Orientation::COUNTERCLOCKWISE);
    }
    {
        const Polygon poly{Vec2(2, 2), Vec2(0, 3), Vec2(0, 0), Vec2(1, -1), Vec2(4, 0), Vec2(4, 2)};
        ASSERT_EQ(poly.orientation(), Orientation::COUNTERCLOCKWISE);
    }
    {
        const Polygon poly{Vec2(0, 0), Vec2(0, 1), Vec2(1, 1), Vec2(1, 0)};
        ASSERT_EQ(poly.orientation(), Orientation::CLOCKWISE);
    }
    {
        const Polygon poly{Vec2(2, 2), Vec2(4, 2), Vec2(4, 0), Vec2(1, -1), Vec2(0, 0), Vec2(0, 3)};
        ASSERT_EQ(poly.orientation(), Orientation::CLOCKWISE);
    }
}

TEST(Polygon, clip) {
    constexpr double eps = 1e-7;
    {
        const Polygon clip_polygon{Vec2(0, 0), Vec2(3, 0), Vec2(3, 3), Vec2(0, 3)};
        const Polygon subject_polygon{
            Vec2(-0.5, 0.5), Vec2(1.5, -1.5), Vec2(2, 0.5), Vec2(2.5, 1.5), Vec2(0.5, 4.5)};

        const Polygon expected_polygon{
            Vec2(0.125, 3),
            Vec2(0, 2.5),
            Vec2(0, 0),
            Vec2(1.875, 0),
            Vec2(2, 0.5),
            Vec2(2.5, 1.5),
            Vec2(1.5, 3)};
        const auto clipped_polygon = clip(clip_polygon, subject_polygon);

        EXPECT_EQ(expected_polygon.size(), clipped_polygon.size());
        for (auto it_1 = expected_polygon.begin(), it_2 = clipped_polygon.begin();
             it_1 != expected_polygon.end();
             ++it_1, ++it_2) {
            ASSERT_GEOM_EQUAL(*it_1, *it_2, eps);
        }
    }
    {
        const Polygon clip_polygon{Vec2(0, 0), Vec2(0, 3), Vec2(3, 3), Vec2(3, 0)};
        const Polygon subject_polygon{
            Vec2(-0.5, 0.5), Vec2(1.5, -1.5), Vec2(2, 0.5), Vec2(2.5, 1.5), Vec2(0.5, 4.5)};

        const Polygon expected_polygon{
            Vec2(0.125, 3),
            Vec2(0, 2.5),
            Vec2(0, 0),
            Vec2(1.875, 0),
            Vec2(2, 0.5),
            Vec2(2.5, 1.5),
            Vec2(1.5, 3)};
        const auto clipped_polygon = clip(clip_polygon, subject_polygon);

        EXPECT_EQ(expected_polygon.size(), clipped_polygon.size());
        for (auto it_1 = expected_polygon.begin(), it_2 = clipped_polygon.begin();
             it_1 != expected_polygon.end();
             ++it_1, ++it_2) {
            ASSERT_GEOM_EQUAL(*it_1, *it_2, eps);
        }
    }
}

TEST(Polygon, intersect) {
    {
        auto poly = Polygon{Vec2(0, 0), Vec2(0, 1), Vec2(1, 0)};
        auto seg = Segment(Vec2(0, 0), Vec2(0, 1));

        ASSERT_EQ(intersect(poly, seg), true);
    }

    {
        auto poly = Polygon{Vec2(0.001, 0), Vec2(0.001, 1), Vec2(1, 0)};
        auto seg = Segment(Vec2(0, 0), Vec2(0, 1));

        ASSERT_EQ(intersect(poly, seg), false);
    }

    {
        auto poly = Polygon{Vec2(0, 0), Vec2(0, 3), Vec2(2, 3), Vec2(2, 0)};
        auto seg = Segment(Vec2(1, 1), Vec2(1, 2));

        ASSERT_EQ(intersect(poly, seg), false);
    }

    {
        auto poly = Polygon{Vec2(0, 0), Vec2(0, 3), Vec2(2, 3), Vec2(2, 0)};
        auto seg = Segment(Vec2(1, 1), Vec2(1, 4));

        ASSERT_EQ(intersect(poly, seg), true);
    }
}
