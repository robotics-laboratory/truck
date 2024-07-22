#include <gtest/gtest.h>

#include "lidar_map/serializer.h"
#include "lidar_map/builder.h"
#include "lidar_map/icp.h"
#include "map/map.h"
#include "geom/distance.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <numeric>

using namespace truck::lidar_map;
using namespace truck::map;
using namespace truck::geom;

const std::string kMapPkgPath = ament_index_cpp::get_package_share_directory("map");

const std::string kScanTopic = "/lidar/scan";
const std::string kOdomTopic = "/ekf/odometry/filtered";

struct Metrics {
    double mean;
    double rmse;
    double q95;
    double q90;
};

/**
 * Metrics for calculations lidar map quality 
 *
 * Error is calculated as distance from point cloud point
 * to the nearest segment of vector map.
 * 
 * Error aggregation is done by next metrics:
 *  mean: mean error
 *  rmse: root mean squared error
 *  q95: 95th quantile
 *  q90: 90th quantile
 */
Metrics calculateMetrics(const Cloud& cloud, const ComplexPolygon& complex_polygon) {
    std::vector<double> min_dists;
    const auto& segments = complex_polygon.segments();

    for (size_t i = 0; i < cloud.features.cols(); i++) {
        const Vec2 cloud_point = {cloud.features.col(i)(0), cloud.features.col(i)(1)};
        double min_dist = distance(cloud_point, segments[0]);

        for (size_t j = 1; j < segments.size(); j++) {
            min_dist = std::min(min_dist, distance(cloud_point, segments[j]));
        }

        min_dists.push_back(min_dist);
    }

    const size_t count = min_dists.size();
    std::sort(min_dists.begin(), min_dists.end());

    auto get_quantile = [&](double q) {
        const size_t id = static_cast<size_t>(q * (count - 1));
        return min_dists[id];
    };

    Metrics metrics = {
        .mean = std::accumulate(
            min_dists.begin(),
            min_dists.end(),
            0.0,
            [&](double a, double b) { return a + b / count; }),

        .rmse = std::sqrt(std::accumulate(
            min_dists.begin(),
            min_dists.end(),
            0.0,
            [&](double a, double b) { return a + (b * b) / count; })),

        .q95 = get_quantile(0.95),
        .q90 = get_quantile(0.90)};

    return metrics;
}

std::ostream& operator<<(std::ostream& out, const Metrics& m) noexcept {
    return out << "mean: \t" << m.mean << "\n"
               << "rmse: \t" << m.rmse << "\n"
               << "q95: \t" << m.q95 << "\n"
               << "q90: \t" << m.q90 << "\n";
}

TEST(LidarMap, lidar_map_test) {
    const std::string kRideBagPath = "test/bags/rides/ride_sim_map_7_2.mcap";
    const std::string kCloudBagPath = "test/bags/clouds/cloud_sim_map_7_2";

    ICPBuilderParams icp_builder_params;

    const BuilderParams builder_params{
        .icp_edge_max_dist = 0.6,
        .poses_min_dist = 0.5,
        .odom_edge_weight = 1.0,
        .icp_edge_weight = 3.0,
        .optimizer_steps = 10,
        .verbose = false};

    ICPBuilder icp_builder = ICPBuilder(icp_builder_params);
    ICP icp = icp_builder.build();

    Builder builder = Builder(builder_params, icp);

    ComplexPolygon vector_map = Map::fromGeoJson(kMapPkgPath + "/data/map_7.geojson").polygons()[0];

    const auto clouds = loadLidarScan(kRideBagPath, kScanTopic);
    const auto poses = loadOdometry(kRideBagPath, kOdomTopic);

    const auto [filtered_poses, filtered_clouds] = builder.filterByPosesProximity(poses, clouds);

    const auto filtered_poses_optimized = builder.optimizePoses(filtered_poses, filtered_clouds);

    const auto lidar_map =
        builder.transformClouds(filtered_poses_optimized, filtered_clouds, true)[0];

    serializeToMCAP(kCloudBagPath, lidar_map, "/lidar_map", vector_map, "/vector_map");

    std::cout << calculateMetrics(lidar_map, vector_map);
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
