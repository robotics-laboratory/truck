#include <gtest/gtest.h>

#include "lidar_map/serializer.h"
#include "lidar_map/builder.h"
#include "lidar_map/icp.h"
#include "map/map.h"

using namespace truck::lidar_map;
using namespace truck::map;
using namespace truck::geom;

TEST(LidarMap, lidar_map_test) {
    const SerializerParams serializer_params{
        .topic = {.odom = "/ekf/odometry/filtered", .point_cloud = "/lidar/scan"},
        .bag_name = {.ride = "ride_real_office_1", .cloud = "cloud_real_office_1"}};

    ICPBuilderParams icp_builder_params;

    const BuilderParams builder_params{
        .icp_edge_max_dist = 0.6,
        .poses_min_dist = 0.5,
        .odom_edge_weight = 1.0,
        .icp_edge_weight = 3.0,
        .optimizer_steps = 10,
        .verbose = false};

    Serializer serializer = Serializer(serializer_params);

    ICPBuilder icp_builder = ICPBuilder(icp_builder_params);
    ICP icp = icp_builder.build();

    Builder builder = Builder(builder_params, icp);

    ComplexPolygon vector_map =
        Map::fromGeoJson("/truck/packages/map/data/map_7.geojson").polygons()[0];

    const auto [poses, clouds] = serializer.deserializeMCAP();
    const auto [filtered_poses, filtered_clouds] = builder.filterByPosesProximity(poses, clouds);

    const auto filtered_poses_optimized = builder.optimizePoses(filtered_poses, filtered_clouds);
    const auto lidar_map =
        builder.transformClouds(filtered_poses_optimized, filtered_clouds, true)[0];

    auto metrics = builder.calculateMetrics(lidar_map, vector_map);

    serializer.serializeToMCAP(lidar_map, "/lidar_map", vector_map, "/vector_map");

    std::cout << "\n\tMetrics for " << serializer_params.bag_name.cloud << "\n";
    std::cout << "\t\tmean: " << metrics["mean"] << "\n";
    std::cout << "\t\trmse: " << metrics["rmse"] << "\n\n";
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
