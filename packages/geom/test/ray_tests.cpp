#include <gtest/gtest.h>

#include "geom/intersection.h"
#include "geom/ray.h"
#include "geom/test/equal_assert.h"

using namespace truck::geom;

TEST(Ray, segment_intersections) {
    constexpr double precision = 1e-4;

    {
        const Vec2 ray_origin(4, 0);
        const Vec2 segment_begin(2, 2);
        const Vec2 segment_end(6, 2);
        const Vec2 correct_intersection(4, 2);
        const AngleVec2 ray_dir(Angle::fromDegrees(90));

        const Ray ray(ray_origin, ray_dir);
        const Segment segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_TRUE(intersection);
        ASSERT_GEOM_EQUAL(*intersection, correct_intersection, precision);
    }

    {
        const Vec2 ray_origin(3, 0);
        const Vec2 segment_begin(2, 2);
        const Vec2 segment_end(6, 2);
        const Vec2 correct_intersection(5, 2);
        const AngleVec2 ray_dir(Angle::fromDegrees(45));

        const Ray ray(ray_origin, ray_dir);
        const Segment segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_TRUE(intersection);
        ASSERT_GEOM_EQUAL(*intersection, correct_intersection, precision);
    }

    {
        const Vec2 ray_origin(3, 0);
        const Vec2 segment_begin(2, 2);
        const Vec2 segment_end(6, 2);
        const AngleVec2 ray_dir(Angle::fromDegrees(135));

        const Ray ray(ray_origin, ray_dir);
        const Segment segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_FALSE(intersection);
    }

    {
        const Vec2 ray_origin(4, 3);
        const Vec2 segment_begin(2, 2);
        const Vec2 segment_end(6, 2);
        const AngleVec2 ray_dir(Angle::fromDegrees(90));

        const Ray ray(ray_origin, ray_dir);
        const Segment segment(segment_begin, segment_end);
        const auto intersection = intersect(ray, segment, precision);

        ASSERT_FALSE(intersection);
    }
}
