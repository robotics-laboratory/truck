#include "simulator_2d/simulation_map.h"

#include <gtest/gtest.h>

using namespace truck::simulator;

TEST(SimulationMap, hasCollision) {
    constexpr double precision = 1e-9;

    {
        // Arrange.
        SimulationMap map;
        map.resetMap("/truck/packages/map/data/map_6.geojson");
        const auto shape = truck::geom::Polygon{{1, 3}, {5, 4}, {7, 5}, {3, 1}};

        // Act.
        const auto result = hasCollision(map, shape, precision);

        // Assert.
        EXPECT_EQ(result, false);
    }

    {
        // Arrange.
        SimulationMap map;
        map.resetMap("/truck/packages/map/data/map_6.geojson");
        const auto shape = truck::geom::Polygon{{25, 32}, {27, 32}, {27, 29}, {25, 29}};

        // Act.
        const auto result = hasCollision(map, shape, precision);

        // Assert.
        EXPECT_EQ(result, true);
    }

    {
        // Arrange.
        SimulationMap map;
        map.resetMap("/truck/packages/map/data/map_6.geojson");
        const auto shape = truck::geom::Polygon{
            {22.133002182962517, 31.000984165385418}, {22, 30}, {21, 31}, {21, 30}};

        // Act.
        const auto result = hasCollision(map, shape, precision);

        // Assert.
        EXPECT_EQ(result, true);
    }
}
