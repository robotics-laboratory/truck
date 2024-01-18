#include <gtest/gtest.h>

#include "fastgrid/distance_transform.h"

#include "geom/test/equal_assert.h"
#include "geom/msg.h"

#include "collision/map.h"

using namespace truck::geom;
using namespace truck::fastgrid;
using namespace truck::collision;
using namespace truck::map;

TEST(CollisionMap, BitMap) {
    constexpr double eps = 1e-7;

    auto collision_map = CollisionMap();

    {
        auto bit_map = collision_map.GetBitMap();
        EXPECT_EQ(bit_map.size.width, 0);
        EXPECT_EQ(bit_map.size.height, 0);
        EXPECT_DOUBLE_EQ(bit_map.resolution, 0.0);
        EXPECT_EQ(bit_map.origin, std::nullopt);
    }

    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.info.resolution = 0.5;
        grid.info.width = 10;
        grid.info.height = 10;
        grid.info.origin = msg::toPose(Pose(Vec2(0, 0), AngleVec2::fromVector(1, 0)));

        std::vector<int8_t> data(100, 1);
        grid.data = data;

        collision_map.SetOccupancyGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid));

        auto bit_map = collision_map.GetBitMap();

        EXPECT_EQ(bit_map.size.width, 10);
        EXPECT_EQ(bit_map.size.height, 10);
        EXPECT_DOUBLE_EQ(bit_map.resolution, 0.5);
        ASSERT_GEOM_EQUAL(bit_map.origin->pose, Pose(Vec2(0, 0), AngleVec2::fromVector(1, 0)));
        EXPECT_EQ(static_cast<int>(bit_map[0][0]), 0);
    }

    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.info.resolution = 1.5;
        grid.info.width = 3;
        grid.info.height = 3;
        grid.info.origin = msg::toPose(Pose(Vec2(1, 1), AngleVec2::fromVector(0, 1)));

        std::vector<int8_t> data(9, 0);
        grid.data = data;

        collision_map.SetOccupancyGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid));

        auto bit_map = collision_map.GetBitMap();

        EXPECT_EQ(bit_map.size.width, 3);
        EXPECT_EQ(bit_map.size.height, 3);
        EXPECT_DOUBLE_EQ(bit_map.resolution, 1.5);
        ASSERT_GEOM_EQUAL(bit_map.origin->pose, Pose(Vec2(1, 1), AngleVec2::fromVector(0, 1)));
        EXPECT_EQ(static_cast<int>(bit_map[0][0]), 1);
    }

    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.info.resolution = 1.0;
        grid.info.width = 3;
        grid.info.height = 3;
        grid.info.origin = msg::toPose(Pose(Vec2(0, 0), AngleVec2::fromVector(1, 0)));

        std::vector<int8_t> data(9, 0);
        grid.data = data;

        collision_map.SetOccupancyGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid));

        ComplexPolygon polygon;
        polygon.outer = {Vec2(1, 1), Vec2(2 - eps, 1), Vec2(2 - eps, 2 - eps), Vec2(1, 2 - eps)};
        auto map = Map({polygon});

        collision_map.SetMap(std::make_shared<Map>(map));

        auto bit_map = collision_map.GetBitMap();

        EXPECT_EQ(static_cast<int>(bit_map[0][0]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[0][1]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[0][2]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[1][0]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[1][1]), 1);
        EXPECT_EQ(static_cast<int>(bit_map[1][2]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[2][0]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[2][1]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[2][2]), 0);
    }

    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.info.resolution = 1.0;
        grid.info.width = 3;
        grid.info.height = 3;
        grid.info.origin = msg::toPose(Pose(Vec2(1, 1), AngleVec2::fromVector(1, 0)));

        std::vector<int8_t> data(9, 0);
        grid.data = data;

        collision_map.SetOccupancyGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid));

        ComplexPolygon polygon;
        polygon.outer = {Vec2(2, 2), Vec2(3, 2), Vec2(3, 3), Vec2(2, 3)};
        auto map = Map({polygon});

        collision_map.SetMap(std::make_shared<Map>(map));

        auto bit_map = collision_map.GetBitMap();

        EXPECT_EQ(static_cast<int>(bit_map[0][0]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[0][1]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[0][2]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[1][0]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[1][1]), 1);
        EXPECT_EQ(static_cast<int>(bit_map[1][2]), 1);
        EXPECT_EQ(static_cast<int>(bit_map[2][0]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[2][1]), 1);
        EXPECT_EQ(static_cast<int>(bit_map[2][2]), 1);
    }

    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.info.resolution = 1.0;
        grid.info.width = 3;
        grid.info.height = 3;
        grid.info.origin = msg::toPose(Pose(Vec2(0, 0), AngleVec2::fromVector(1, 0)));

        std::vector<int8_t> data(9, 0);
        grid.data = data;

        collision_map.SetOccupancyGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid));

        ComplexPolygon polygon;
        polygon.inners.push_back(
            {Vec2(0, 0), Vec2(0, 1 - eps), Vec2(1 - eps, 1 - eps), Vec2(1 - eps, 0)});
        polygon.inners.push_back(
            {Vec2(1, 1), Vec2(1, 2 - eps), Vec2(2 - eps, 2 - eps), Vec2(2 - eps, 1)});

        auto map = Map({polygon});

        collision_map.SetMap(std::make_shared<Map>(map));

        auto bit_map = collision_map.GetBitMap();

        EXPECT_EQ(static_cast<int>(bit_map[0][0]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[0][1]), 1);
        EXPECT_EQ(static_cast<int>(bit_map[0][2]), 1);
        EXPECT_EQ(static_cast<int>(bit_map[1][0]), 1);
        EXPECT_EQ(static_cast<int>(bit_map[1][1]), 0);
        EXPECT_EQ(static_cast<int>(bit_map[1][2]), 1);
        EXPECT_EQ(static_cast<int>(bit_map[2][0]), 1);
        EXPECT_EQ(static_cast<int>(bit_map[2][1]), 1);
        EXPECT_EQ(static_cast<int>(bit_map[2][2]), 1);
    }
}

TEST(CollisionMap, DistanceMap) {
    constexpr double eps = 1e-7;

    auto collision_map = CollisionMap();

    nav_msgs::msg::OccupancyGrid grid;
    grid.info.resolution = 1.0;
    grid.info.width = 3;
    grid.info.height = 3;
    grid.info.origin = msg::toPose(Pose(Vec2(0, 0), AngleVec2::fromVector(1, 0)));

    std::vector<int8_t> data(9, 0);
    data[0] = 1;
    grid.data = data;

    collision_map.SetOccupancyGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid));

    ComplexPolygon polygon;
    polygon.inners.push_back(
        {Vec2(2, 2), Vec2(2, 3 - eps), Vec2(3 - eps, 3 - eps), Vec2(3 - eps, 2)});

    auto map = Map({polygon});

    collision_map.SetMap(std::make_shared<Map>(map));

    auto distance_map = collision_map.GetDistanceMap();

    auto occupancy_grid = makeGrid<uint8_t>(
        Size{.width = 3, .height = 3}, 1.0, Pose(Vec2(0, 0), AngleVec2::fromVector(1, 0)));
    auto distance_transform = distanceTransformApprox3(occupancy_grid.grid);

    equal(distance_map, *distance_transform, eps);
}
