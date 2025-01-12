#include "cmath"
#include "geom/distance.h"
#include "geom/msg.h"
#include "lidar_map/builder.h"
#include "lidar_map/conversion.h"
#include "lidar_map/serialization.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/program_options.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace truck::lidar_map;
using namespace truck::geom;

namespace po = boost::program_options;

<<<<<<< HEAD
const std::string kInputTopicPointCloud = "/livox/lidar";
const std::string kInputTopicOdom = "/ekf/odometry/filtered";
const std::string kOutputTopicLidarMap = "/lidar_map";
=======
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
    std::vector<double> min_dists_squared;
    const auto& segments = complex_polygon.segments();

    for (int i = 0; i < cloud.cols(); i++) {
        const Vec2 cloud_point = {cloud.col(i)(0), cloud.col(i)(1)};
        double min_dist_squared = distanceSq(cloud_point, segments[0]);

        for (size_t j = 1; j < segments.size(); j++) {
            min_dist_squared = std::min(min_dist_squared, distanceSq(cloud_point, segments[j]));
        }

        min_dists_squared.push_back(min_dist_squared);
    }

    const size_t count = min_dists_squared.size();
    std::vector<double> min_dists(count, 0);

    for (size_t i = 0; i < count; i++) {
        min_dists[i] = std::sqrt(min_dists_squared[i]);
    }

    std::sort(min_dists.begin(), min_dists.end());

    auto get_quantile = [&](double q) {
        const auto id = static_cast<size_t>(q * (count - 1));
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

const std::string notebookPath = "/notebook";
const std::string kTopicLaserScan = "/livox/lidar";
const std::string kTopicOdom = "/ekf/odometry/filtered";
const std::string kPkgPathMap = ament_index_cpp::get_package_share_directory("map");
>>>>>>> f86400c (add notebook + fix formatting)
const std::string kPkgPathLidarMap = ament_index_cpp::get_package_share_directory("lidar_map");

int main(int argc, char* argv[]) {
    bool enable_mcap_log = false;
    bool enable_json_log = false;

    std::string vector_map_file;
    std::string mcap_input_path;
    std::string mcap_output_folder_path;
    std::string mcap_log_folder_path;
    std::string json_log_path;
    std::string icp_log_path;
    int icp_cloud_number;
    int normals_rarefaction_percentage;

    {
        po::options_description desc("Executable for constructing 2D LiDAR map");
        desc.add_options()("help,h", "show this help message and exit")(
            "mcap-input",
            po::value<std::string>(&mcap_input_path)->required(),
            "path to .mcap file with a ride")(
            "mcap-output",
            po::value<std::string>(&mcap_output_folder_path)->required(),
            "path to NON-EXISTING folder for saving map")(
            "mcap-log",
            po::value<std::string>(&mcap_log_folder_path)->default_value(""),
            "path to NON-EXISTING folder for saving map logs")(
            "json-log",
            po::value<std::string>(&json_log_path)->default_value(""),
            "path to json file for saving map logs")(
            "icp-log,il",
            po::value<std::string>(&icp_log_path)->default_value(""),
            "pathto NON-EXISTING folder for saving ICP log file (normals and outliers)")(
            "cloud_number,cn",
            po::value<int>(&icp_cloud_number)->default_value(1),
            "cloud number for visualizing normals and outliers for icp-log")(
            "percentage,p",
            po::value<int>(&normals_rarefaction_percentage)->default_value(100),
            "percentage of normals rendered [0;100] for icp-log");

        po::variables_map vm;
        try {
            po::store(po::parse_command_line(argc, argv, desc), vm);

            if (vm.count("help")) {
                std::cout << desc << "\n";
                return 0;
            }

            po::notify(vm);
        } catch (po::error& e) {
            std::cerr << "ERROR: " << e.what() << "\n";
            std::cerr << desc << "\n";
            return 1;
        }

        if (!mcap_log_folder_path.empty()) {
            enable_mcap_log = true;
        }
        if (!json_log_path.empty()) {
            enable_json_log = true;
        }
    }

    const BuilderParams builder_params{
        .icp_config = kPkgPathLidarMap + "/config/icp.yaml",
        .icp_edge_max_dist = 3,
        .odom_edge_weight = 1.0,
        .icp_edge_weight = 3.0,
        .verbose = true};

    Builder builder = Builder(builder_params);

    const size_t optimization_steps = 0;

    writer::MCAPWriterParams mcap_writer_params = {
        .mcap_path = mcap_log_folder_path,
        .poses_topic_name = "/poses",
        .cloud_topic_name = "/cloud",
    };

    writer::MCAPWriter mcap_writer(mcap_writer_params);

    Poses poses;
    Clouds clouds;

    // 1. Read data from input bag
    {
        auto odom_msgs = reader::readOdomTopic(mcap_input_path, kInputTopicOdom);

        auto point_cloud_msgs = reader::readPointCloudTopic(mcap_input_path, kInputTopicPointCloud);

        const auto [synced_odom_msgs, synced_point_cloud_msgs] =
            syncOdomWithPointCloud(odom_msgs, point_cloud_msgs);

        const auto all_poses = toPoses(synced_odom_msgs);

        const auto all_clouds = toClouds(synced_point_cloud_msgs);

        std::tie(poses, clouds) = builder.sliceDataByPosesProximity(all_poses, all_clouds, 8.0);

        const auto rotate_poses_by_angle_PI = [](auto& poses) {
            for (auto& pose : poses) {
                pose.dir += truck::geom::Angle::fromRadians(M_PI);
            }
        };

        // TODO (apmilko): fix clouds orientation
        rotate_poses_by_angle_PI(poses);
    }

    // 2. Construct and optimize pose graph
    {
        builder.initPoseGraph(poses, clouds, !icp_log_path.empty());

        if (enable_mcap_log) {
            const Cloud lidar_map_on_iteration =
                builder.mergeClouds(builder.transformClouds(poses, clouds));

            mcap_writer.writeCloud(lidar_map_on_iteration);
            mcap_writer.writePoses(poses);
            mcap_writer.update();
        }

        if (enable_json_log) {
            const auto pose_graph_info = builder.calculatePoseGraphInfo();
            writer::writePoseGraphInfoToJSON(json_log_path, pose_graph_info, 0);
        }

        for (size_t i = 1; i <= optimization_steps; i++) {
            poses = builder.optimizePoseGraph();

            if (enable_mcap_log) {
                const Cloud lidar_map_on_iteration =
                    builder.mergeClouds(builder.transformClouds(poses, clouds));

                mcap_writer.writeCloud(lidar_map_on_iteration);
                mcap_writer.writePoses(poses);
                mcap_writer.update();
            }

            if (enable_json_log) {
                const auto pose_graph_info = builder.calculatePoseGraphInfo();
                writer::writePoseGraphInfoToJSON(json_log_path, pose_graph_info, i);
    
            const size_t optimization_steps = 10;
            if (enable_log) {
                log_optimization_step();
                const PoseGraphInfo pose_graph_info = builder.calculatePoseGraphInfo();
                const std::string pose_graph_info_path =
                    notebookPath + "/" + kposeGraphInfoJSON;
                builder.writePoseGraphInfoToJSON(pose_graph_info_path, pose_graph_info, 0);
            }

            for (size_t i = 0; i < optimization_steps; i++) {
                poses = builder.optimizePoseGraph(1);

                if (enable_log) {
                    log_optimization_step();
                    const PoseGraphInfo pose_graph_info = builder.calculatePoseGraphInfo();
                    const std::string pose_graph_info_path =
                        notebookPath + "/" + kposeGraphInfoJSON;
                    builder.writePoseGraphInfoToJSON(pose_graph_info_path, pose_graph_info, i + 1);
                }
            }

            clouds = builder.applyGridFilter(clouds, 0.08);

            if (enable_log) {
                log_optimization_step();
            }

            clouds = builder.applyDynamicFilter(poses, clouds, 12.0, 1, 0.05);

            if (enable_log) {
                log_optimization_step();
            }

            const auto lidar_map = builder.mergeClouds(builder.transformClouds(poses, clouds));
            
            bag_writer.addLidarMap(lidar_map, "/map/lidar");

            if (enable_test) {
                const std::string map_path = kPkgPathMap + "/data/" + vector_map_file;
                const ComplexPolygon vector_map = Map::fromGeoJson(map_path).polygons()[0];

                bag_writer.addVectorMap(vector_map, "/map/vector");
                std::cout << calculateMetrics(lidar_map, vector_map);
            }
        }

        clouds = builder.applyGridFilter(clouds);

        const auto lidar_map = builder.mergeClouds(builder.transformClouds(poses, clouds));

        if (!icp_log_path.empty()) {
            mcap_writer.writeCloudWithAttributes(
                icp_log_path,
                builder.clouds_with_attributes[icp_cloud_number],
                normals_rarefaction_percentage);
        }

        writer::MCAPWriter::writeCloud(mcap_output_folder_path, lidar_map, kOutputTopicLidarMap);
    }
}
