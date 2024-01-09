#include <gtest/gtest.h>

#include "geom/test/equal_assert.h"

#include "map/map_builder.h"

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <iostream>

using namespace truck::geom;
using namespace truck::map;
using namespace truck::fastgrid;

TEST(Map, clip) {
    {
        const Size size = {3, 3};
        const double resolution = 1.0;
        const Pose pose = {{0, 0}, AngleVec2::fromVector(1, 0)};

        const auto grid = Grid<uint8_t>(size, resolution, pose);
        const auto domain = Domain(grid);

        const ComplexPolygon poly({Vec2(0, 1), Vec2(2, -1), Vec2(2.5, 1), Vec2(3, 2), Vec2(1, 5)});

        auto map = Map({poly});

        const auto clipped_polygons = map.clip(domain);

        EXPECT_EQ(clipped_polygons.size(), 1);
        const ComplexPolygon expected_poly(
            {Vec2(0.625, 3.5),
             Vec2(0.5, 3),
             Vec2(0.5, 0.5),
             Vec2(2.375, 0.5),
             Vec2(2.5, 1.0),
             Vec2(3.0, 2.0),
             Vec2(2.0, 3.5)});
        EXPECT_EQ(clipped_polygons[0].outer.size(), expected_poly.outer.size());
        for (size_t i = 0; i < expected_poly.outer.size(); ++i) {
            ASSERT_GEOM_EQUAL(clipped_polygons[0].outer[i], expected_poly.outer[i]);
        }
        EXPECT_TRUE(clipped_polygons[0].inners.empty());
    }
    {
        const Size size = {3, 3};
        const double resolution = 1.0;
        const Pose pose = {{0, 0}, AngleVec2::fromVector(0, 1)};

        const auto grid = Grid<uint8_t>(size, resolution, pose);
        const auto domain = Domain(grid);

        const ComplexPolygon poly({Vec2(-2, 0), Vec2(0, 2), Vec2(-2, 4), Vec2(-4, 2)});

        auto map = Map({poly});

        const auto clipped_polygons = map.clip(domain);

        EXPECT_EQ(clipped_polygons.size(), 1);
        const ComplexPolygon expected_poly(
            {Vec2(-3.5, 1.5),
             Vec2(-2.5, 0.5),
             Vec2(-1.5, 0.5),
             Vec2(-0.5, 1.5),
             Vec2(-0.5, 2.5),
             Vec2(-1.5, 3.5),
             Vec2(-2.5, 3.5),
             Vec2(-3.5, 2.5)});
        EXPECT_EQ(clipped_polygons[0].outer.size(), expected_poly.outer.size());
        for (size_t i = 0; i < expected_poly.outer.size(); ++i) {
            ASSERT_GEOM_EQUAL(clipped_polygons[0].outer[i], expected_poly.outer[i]);
        }
        EXPECT_TRUE(clipped_polygons[0].inners.empty());
    }
}