#include <gtest/gtest.h>

#include "geom/intersection.h"
#include "geom/ray.h"
#include "geom/test/equal_assert.h"

using namespace truck::geom;

TEST(Ray, segment_intersections) {
    constexpr double precision = 1e-4;

    {
        Vec2 ray_origin(4, 0);
        Vec2 segment_begin(2, 2);
        Vec2 segment_end(6, 2);
        Vec2 correct_intersection(4, 2);
        AngleVec2 const ray_dir(Angle::fromDegrees(90));

        Ray const ray(ray_origin, ray_dir);
        Segment const segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_TRUE(intersection);
        ASSERT_GEOM_EQUAL(*intersection, correct_intersection, precision);
    }

    {
        Vec2 ray_origin(3, 0);
        Vec2 segment_begin(2, 2);
        Vec2 segment_end(6, 2);
        Vec2 correct_intersection(5, 2);
        AngleVec2 const ray_dir(Angle::fromDegrees(45));

        Ray const ray(ray_origin, ray_dir);
        Segment const segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_TRUE(intersection);
        ASSERT_GEOM_EQUAL(*intersection, correct_intersection, precision);
    }

    {
        Vec2 ray_origin(3, 0);
        Vec2 segment_begin(2, 2);
        Vec2 segment_end(6, 2);
        AngleVec2 const ray_dir(Angle::fromDegrees(135));

        Ray const ray(ray_origin, ray_dir);
        Segment const segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_FALSE(intersection);
    }

    {
        Vec2 ray_origin(4, 3);
        Vec2 segment_begin(2, 2);
        Vec2 segment_end(6, 2);
        AngleVec2 const ray_dir(Angle::fromDegrees(90));

        Ray const ray(ray_origin, ray_dir);
        Segment const segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_FALSE(intersection);
    }
}
