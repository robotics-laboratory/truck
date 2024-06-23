#include "simulator_2d/simulation_map.h"

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace truck::simulator;

const std::string kMapPkgPath = ament_index_cpp::get_package_share_directory("map");

TEST(SimulationMap, hasCollision) {
    constexpr double precision = 1e-9;

    {
        // Arrange.
        SimulationMap map;
        map.resetMap(kMapPkgPath + "/data/map_6.geojson");
        const auto shape = truck::geom::Polygon{{20, 38}, {19, 38}, {20, 37}, {19, 37}};

        // Act.
        const auto result = hasCollision(map, shape, precision);

        // Assert.
        EXPECT_EQ(result, false);
    }

    {
        // Arrange.
        SimulationMap map;
        map.resetMap(kMapPkgPath + "/data/map_6.geojson");
        const auto shape = truck::geom::Polygon{{25, 32}, {27, 32}, {27, 29}, {25, 29}};

        // Act.
        const auto result = hasCollision(map, shape, precision);

        // Assert.
        EXPECT_EQ(result, true);
    }

    {
        // Arrange.
        SimulationMap map;
        map.resetMap(kMapPkgPath + "/data/map_6.geojson");
        const auto shape = truck::geom::Polygon{
            {22.133002182962517, 31.000984165385418}, {22, 30}, {21, 31}, {21, 30}};

        // Act.
        const auto result = hasCollision(map, shape, precision);

        // Assert.
        EXPECT_EQ(result, true);
    }
}
