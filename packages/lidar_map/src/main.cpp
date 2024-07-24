#include "lidar_map/serializer.h"
#include "lidar_map/builder.h"
#include "lidar_map/icp.h"
#include "lidar_map/conversion.h"
#include "map/map.h"
#include "geom/msg.h"
#include "geom/distance.h"

#include <numeric>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/program_options.hpp>

using namespace truck::lidar_map;
using namespace truck::map;
using namespace truck::geom;

namespace po = boost::program_options;

namespace {

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

    for (size_t i = 0; i < cloud.cols(); i++) {
        const Vec2 cloud_point = {cloud.col(i)(0), cloud.col(i)(1)};
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
    return out << "Metrics:\n"
               << "  mean = " << m.mean << "\n"
               << "  rmse = " << m.rmse << "\n"
               << "  q95 = " << m.q95 << "\n"
               << "  q90 = " << m.q90 << "\n";
}

}  // namespace

bool kTest;
std::string kVectorMapName;
std::string kInputMCAP;
std::string kOutputFolder;

const std::string kLaserScanTopic = "/lidar/scan";
const std::string kOdomTopic = "/ekf/odometry/filtered";
const std::string kMapPkgPath = ament_index_cpp::get_package_share_directory("map");

void buildMap() {
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

    auto odom_msgs = loadOdomTopic(kInputMCAP, kOdomTopic);
    auto laser_scan_msgs = loadLaserScanTopic(kInputMCAP, kLaserScanTopic);

    syncOdomWithCloud(odom_msgs, laser_scan_msgs);

    const auto all_poses = toPoses(odom_msgs);
    const auto all_clouds = toClouds(laser_scan_msgs);

    const auto [poses, clouds] = builder.filterByPosesProximity(all_poses, all_clouds);

    const auto poses_optimized = builder.optimizePoses(poses, clouds);

    const auto clouds_tf = builder.transformClouds(poses_optimized, clouds);
    const auto final_cloud = builder.mergeClouds(clouds_tf);

    if (kTest) {
        ComplexPolygon vector_map =
            Map::fromGeoJson(kMapPkgPath + "/data/" + kVectorMapName).polygons()[0];
        writeToMCAP(kOutputFolder, final_cloud, "/cloud", vector_map, "/map");
        std::cout << calculateMetrics(final_cloud, vector_map);
    } else {
        writeToMCAP(kOutputFolder, final_cloud, "/cloud");
    }
}

int main(int argc, char* argv[]) {
    {
        po::options_description desc("Options and arguments");
        desc.add_options()("help,h", "show this help message and exit")(
            "input,i",
            po::value<std::string>(&kInputMCAP)->required(),
            "path to .mcap file with ride bag")(
            "output,o",
            po::value<std::string>(&kOutputFolder)->required(),
            "path to folder where to store .mcap file with 2D LiDAR map (folder shouldn't exist)")(
            "test,t",
            po::value<std::string>(&kVectorMapName),
            "enable test mode and set vector map (only for simulated data)");

        po::variables_map vm;
        try {
            po::store(po::parse_command_line(argc, argv, desc), vm);

            if (vm.count("help")) {
                std::cout << desc << "\n";
                return 0;
            }

            if (vm.count("test")) {
                kTest = true;
            }

            po::notify(vm);
        } catch (po::error& e) {
            std::cerr << "ERROR: " << e.what() << "\n";
            std::cerr << desc << "\n";
            return 1;
        }
    }

    buildMap();
    return 0;
}
