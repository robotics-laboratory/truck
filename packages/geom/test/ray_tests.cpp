#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"
#include "geom/intersection.h"
#include "geom/ray.h"

using namespace truck::geom;

TEST(Ray, segment_intersections) {
    using namespace truck::geom;

    constexpr double precision = 1e-4;

    {
        Vec2 ray_origin(4, 0), segment_begin(2, 2), segment_end(6, 2), correct_intersection(4, 2);
        AngleVec2 ray_dir(Angle::fromDegrees(90));

        Ray ray(ray_origin, ray_dir);
        Segment segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_TRUE(intersection);
        ASSERT_GEOM_EQUAL(*intersection, correct_intersection, precision);
    }

    {
        Vec2 ray_origin(3, 0), segment_begin(2, 2), segment_end(6, 2), correct_intersection(5, 2);
        AngleVec2 ray_dir(Angle::fromDegrees(45));

        Ray ray(ray_origin, ray_dir);
        Segment segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_TRUE(intersection);
        ASSERT_GEOM_EQUAL(*intersection, correct_intersection, precision);
    }

    {
        Vec2 ray_origin(3, 0), segment_begin(2, 2), segment_end(6, 2);
        AngleVec2 ray_dir(Angle::fromDegrees(135));

        Ray ray(ray_origin, ray_dir);
        Segment segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_FALSE(intersection);
    }

    {
        Vec2 ray_origin(4, 3), segment_begin(2, 2), segment_end(6, 2);
        AngleVec2 ray_dir(Angle::fromDegrees(90));

        Ray ray(ray_origin, ray_dir);
        Segment segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_FALSE(intersection);
    }
}
