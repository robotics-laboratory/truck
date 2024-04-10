#include <gtest/gtest.h>

#include "collision/collision_checker.h"

#include "fastgrid/distance_transform.h"

#include "geom/test/equal_assert.h"
#include "geom/msg.h"

using namespace truck;
using namespace truck::geom;
using namespace truck::collision;
using namespace truck::map;

TEST(StaticCollisionChecker, BitMap) {
    constexpr double eps = 1e-7;

    {
        auto collision_map = StaticCollisionChecker({}, model::Shape());
        auto bit_map = collision_map.BitMap();
        EXPECT_EQ(bit_map.size.width, 401);
        EXPECT_EQ(bit_map.size.height, 401);
        EXPECT_DOUBLE_EQ(bit_map.resolution, 0.1);
        EXPECT_EQ(bit_map.origin, std::nullopt);
    }

    {
        const auto ego_pose = Pose(Vec2(2.5, 2.5), AngleVec2::fromVector(0, 1));
        const double radius = 2.5;
        const double resolution = 0.5;
        const int width = 2 * ceil<int>(radius / resolution) + 1;
        const int height = 2 * ceil<int>(radius / resolution) + 1;
        const auto origin_pose = Pose(
            ego_pose.pos - ego_pose.dir * radius - ego_pose.dir.right() * radius,
            ego_pose.dir.right());

        nav_msgs::msg::OccupancyGrid grid;
        grid.info.resolution = resolution;
        grid.info.width = width;
        grid.info.height = height;
        grid.info.origin = msg::toPose(origin_pose);

        std::vector<int8_t> data(grid.info.height * grid.info.width, 100);
        grid.data = data;

        auto collision_map =
            StaticCollisionChecker({.radius = radius, .resolution = resolution}, model::Shape());
        collision_map.SetOccupancyGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid))
            .SetEgoPose(ego_pose);

        auto bit_map = collision_map.BitMap();

        EXPECT_EQ(bit_map.size.width, width);
        EXPECT_EQ(bit_map.size.height, height);
        EXPECT_DOUBLE_EQ(bit_map.resolution, resolution);
        ASSERT_GEOM_EQUAL(bit_map.origin->pose, origin_pose);
        for (int i = 0; i < bit_map.size.height; ++i) {
            for (int j = 0; j < bit_map.size.width; ++j) {
                EXPECT_EQ(bit_map[i][j], 100);
            }
        }
    }

    {
        const auto ego_pose = Pose(Vec2(1.5, 1.5), AngleVec2::fromVector(0, 1));
        const double radius = 1.5;
        const double resolution = 1.0;
        const int width = 2 * ceil<int>(radius / resolution) + 1;
        const int height = 2 * ceil<int>(radius / resolution) + 1;
        const auto origin_pose = Pose(
            ego_pose.pos - ego_pose.dir * radius - ego_pose.dir.right() * radius,
            ego_pose.dir.right());

        nav_msgs::msg::OccupancyGrid grid;
        grid.info.resolution = resolution;
        grid.info.width = width;
        grid.info.height = height;
        grid.info.origin = msg::toPose(origin_pose);

        std::vector<int8_t> data(grid.info.width * grid.info.height, 0);
        grid.data = data;

        auto collision_map =
            StaticCollisionChecker({.radius = radius, .resolution = resolution}, model::Shape());
        collision_map.SetOccupancyGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid))
            .SetEgoPose(ego_pose);

        ComplexPolygon polygon;
        polygon.outer = {Vec2(1, 1), Vec2(2 + eps, 1), Vec2(2 + eps, 2 + eps), Vec2(1, 2 + eps)};
        auto map = Map({polygon});

        collision_map.SetMap(std::make_shared<Map>(map));

        auto bit_map = collision_map.BitMap();

        EXPECT_EQ(bit_map[0][0], 0);
        EXPECT_EQ(bit_map[0][1], 0);
        EXPECT_EQ(bit_map[0][2], 0);
        EXPECT_EQ(bit_map[1][0], 0);
        EXPECT_EQ(bit_map[1][1], 100);
        EXPECT_EQ(bit_map[1][2], 100);
        EXPECT_EQ(bit_map[2][0], 0);
        EXPECT_EQ(bit_map[2][1], 100);
        EXPECT_EQ(bit_map[2][2], 100);
    }

    {
        const auto ego_pose = Pose(Vec2(1.5, 1.5), AngleVec2::fromVector(0, 1));
        const double radius = 1.5;
        const double resolution = 1.0;
        const auto origin_pose = Pose(
            ego_pose.pos - ego_pose.dir * radius - ego_pose.dir.right() * radius,
            ego_pose.dir.right());

        nav_msgs::msg::OccupancyGrid grid;
        grid.info.resolution = resolution - eps;
        grid.info.width = 3;
        grid.info.height = 3;
        grid.info.origin = msg::toPose(origin_pose);

        std::vector<int8_t> data(grid.info.width * grid.info.height, 100);

        grid.data = data;

        auto collision_map =
            StaticCollisionChecker({.radius = radius, .resolution = resolution}, model::Shape());
        collision_map.SetOccupancyGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid));

        ComplexPolygon polygon;
        polygon.outer = {Vec2(2, 2), Vec2(2, 5), Vec2(5, 5), Vec2(5, 2)};
        polygon.inners = {{Vec2(3 - eps, 3 - eps), Vec2(3 - eps, 4), Vec2(4, 4), Vec2(4, 3 - eps)}};

        auto map = Map({polygon});

        collision_map.SetMap(std::make_shared<Map>(map));

        auto bit_map = collision_map.SetEgoPose(ego_pose).BitMap();

        EXPECT_EQ(bit_map[0][0], 100);
        EXPECT_EQ(bit_map[0][1], 100);
        EXPECT_EQ(bit_map[0][2], 100);
        EXPECT_EQ(bit_map[0][3], 0);
        EXPECT_EQ(bit_map[0][4], 0);
        EXPECT_EQ(bit_map[1][0], 100);
        EXPECT_EQ(bit_map[1][1], 100);
        EXPECT_EQ(bit_map[1][2], 100);
        EXPECT_EQ(bit_map[1][3], 0);
        EXPECT_EQ(bit_map[1][4], 0);
        EXPECT_EQ(bit_map[2][0], 100);
        EXPECT_EQ(bit_map[2][1], 100);
        EXPECT_EQ(bit_map[2][2], 100);
        EXPECT_EQ(bit_map[2][3], 100);
        EXPECT_EQ(bit_map[2][4], 100);
        EXPECT_EQ(bit_map[3][0], 0);
        EXPECT_EQ(bit_map[3][1], 0);
        EXPECT_EQ(bit_map[3][2], 100);
        EXPECT_EQ(bit_map[3][3], 0);
        EXPECT_EQ(bit_map[3][4], 100);
        EXPECT_EQ(bit_map[4][0], 0);
        EXPECT_EQ(bit_map[4][1], 0);
        EXPECT_EQ(bit_map[4][2], 100);
        EXPECT_EQ(bit_map[4][3], 100);
        EXPECT_EQ(bit_map[4][4], 100);
    }
}

TEST(StaticCollisionChecker, DistanceMap) {
    constexpr double eps = 1e-7;

    const auto ego_pose = Pose(Vec2(1.5, 1.5), AngleVec2::fromVector(0, 1));
    const double radius = 1.5;
    const double resolution = 1.0;
    const auto origin_pose = Pose(
        ego_pose.pos - ego_pose.dir * radius - ego_pose.dir.right() * radius, ego_pose.dir.right());

    nav_msgs::msg::OccupancyGrid grid;
    grid.info.resolution = resolution - eps;
    grid.info.width = 3;
    grid.info.height = 3;
    grid.info.origin = msg::toPose(origin_pose);

    std::vector<int8_t> data(grid.info.width * grid.info.height, 100);

    grid.data = data;

    auto collision_map =
        StaticCollisionChecker({.radius = radius, .resolution = resolution}, model::Shape());
    collision_map.SetOccupancyGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>(grid));

    ComplexPolygon polygon;
    polygon.outer = {Vec2(2, 2), Vec2(2, 5), Vec2(5, 5), Vec2(5, 2)};
    polygon.inners = {{Vec2(3 - eps, 3 - eps), Vec2(3 - eps, 4), Vec2(4, 4), Vec2(4, 3 - eps)}};

    auto map = Map({polygon});

    collision_map.SetMap(std::make_shared<Map>(map));

    auto distance_map = collision_map.SetEgoPose(ego_pose).DistanceMap();

    auto occupancy_grid = fastgrid::makeGrid<uint8_t>(
        fastgrid::Size{.width = 5, .height = 5},
        1.0,
        Pose(Vec2(0, 0), AngleVec2::fromVector(1, 0)));
    occupancy_grid.grid.SetTo(0);
    occupancy_grid.grid[0][0] = 100;
    occupancy_grid.grid[0][1] = 100;
    occupancy_grid.grid[0][2] = 100;
    occupancy_grid.grid[0][3] = 0;
    occupancy_grid.grid[0][4] = 0;
    occupancy_grid.grid[1][0] = 100;
    occupancy_grid.grid[1][1] = 100;
    occupancy_grid.grid[1][2] = 100;
    occupancy_grid.grid[1][3] = 0;
    occupancy_grid.grid[1][4] = 0;
    occupancy_grid.grid[2][0] = 100;
    occupancy_grid.grid[2][1] = 100;
    occupancy_grid.grid[2][2] = 100;
    occupancy_grid.grid[2][3] = 100;
    occupancy_grid.grid[2][4] = 100;
    occupancy_grid.grid[3][0] = 0;
    occupancy_grid.grid[3][1] = 0;
    occupancy_grid.grid[3][2] = 100;
    occupancy_grid.grid[3][3] = 0;
    occupancy_grid.grid[3][4] = 100;
    occupancy_grid.grid[4][0] = 0;
    occupancy_grid.grid[4][1] = 0;
    occupancy_grid.grid[4][2] = 100;
    occupancy_grid.grid[4][3] = 100;
    occupancy_grid.grid[4][4] = 100;

    auto distance_transform = distanceTransformApprox3(occupancy_grid.grid);

    equal(distance_map, *distance_transform, eps);
}
