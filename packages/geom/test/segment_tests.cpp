#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/intersection.h"
#include "geom/segment.h"

using namespace truck::geom;

TEST(Segment, intersect) {
    {
        auto seg1 = Segment(Vec2(0, 0), Vec2(0, 1));
        auto seg2 = Segment(Vec2(1, 0), Vec2(1, 1));

        ASSERT_EQ(intersect(seg1, seg2), false);
    }

    {
        auto seg1 = Segment(Vec2(0, 0), Vec2(1, 0));
        auto seg2 = Segment(Vec2(0, 1), Vec2(1, 1));

        ASSERT_EQ(intersect(seg1, seg2), false);
    }

    {
        auto seg1 = Segment(Vec2(0, 0), Vec2(0, 1));
        auto seg2 = Segment(Vec2(0, 1), Vec2(1, 1));

        ASSERT_EQ(intersect(seg1, seg2), true);
    }

    {
        auto seg1 = Segment(Vec2(0, 0), Vec2(0, 1));
        auto seg2 = Segment(Vec2(-1, 1), Vec2(1, 1));

        ASSERT_EQ(intersect(seg1, seg2), true);
    }

    {
        auto seg1 = Segment(Vec2(0, 0), Vec2(0, 1));
        auto seg2 = Segment(Vec2(-1, 2), Vec2(1, 1));

        ASSERT_EQ(intersect(seg1, seg2), false);
    }

    {
        auto seg1 = Segment(Vec2(1, 0), Vec2(1, 2));
        auto seg2 = Segment(Vec2(0, 1), Vec2(2, 1));

        ASSERT_EQ(intersect(seg1, seg2), true);
    }

    {
        auto seg1 = Segment(Vec2(0, 0), Vec2(0, 1));
        auto seg2 = Segment(Vec2(0, 0), Vec2(0, 1));

        ASSERT_EQ(intersect(seg1, seg2), true);
    }
}
