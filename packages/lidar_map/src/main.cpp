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

const std::string kInputTopicPointCloud = "/livox/lidar";
const std::string kInputTopicOdom = "/ekf/odometry/filtered";
const std::string kOutputTopicLidarMap = "/lidar_map";
const std::string kOutputTopicPoses = "/poses";
const std::string kOutputTopicClouds = "/clouds";
const std::string kPkgPathLidarMap = ament_index_cpp::get_package_share_directory("lidar_map");
const std::string kIcpConfigPath = kPkgPathLidarMap + "/config/icp.yaml";
const int kPointsIntensityThreshold = 15;
const double kMinPosesDist = 1.0;
const double kIcpEdgeMaxDist = 3.0;
const double kIcpEdgeMinDist = 1.0;
const double kOdomEdgeWeight = 1.0;
const double kIcpEdgeWeight = 3.0;
const size_t kOptimizationSteps = 10;

int main(int argc, char* argv[]) {
    bool enable_mcap_log = false;
    bool enable_json_log = false;

    std::string vector_map_file;
    std::string mcap_input_path;
    std::string mcap_output_folder_path;
    std::string mcap_log_folder_path;
    std::string json_log_path;

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
            "path to json file for saving map logs");

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
        .icp_config = kIcpConfigPath,
        .icp_edge_max_dist = kIcpEdgeMaxDist,
        .icp_edge_min_dist = kIcpEdgeMinDist,
        .odom_edge_weight = kOdomEdgeWeight,
        .icp_edge_weight = kIcpEdgeWeight,
        .min_poses_dist = kMinPosesDist,
        .verbose = true};

    Builder builder = Builder(builder_params);

    std::shared_ptr<serialization::writer::MCAPWriter> mcap_writer = nullptr;

    if (enable_mcap_log) {
        mcap_writer = std::make_shared<serialization::writer::MCAPWriter>(
            serialization::writer::MCAPWriterParams{
                .mcap_path = mcap_log_folder_path,
                .poses_topic_name = kOutputTopicPoses,
                .cloud_topic_name = kOutputTopicClouds});
    }

    Poses poses;
    Clouds clouds;

    // 1. Read data from input bag
    {
        const auto [odom_msgs, point_cloud_msgs] =
            serialization::reader::readAndSyncOdomWithPointCloud(
                mcap_input_path, kInputTopicOdom, kInputTopicPointCloud, kMinPosesDist);

        poses = toPoses(odom_msgs);
        clouds = toClouds(point_cloud_msgs);

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
        builder.initPoseGraph(poses, clouds);

        if (enable_mcap_log) {
            const Cloud lidar_map_on_iteration =
                builder.mergeClouds(builder.transformClouds(poses, clouds));

            mcap_writer->writeCloud(lidar_map_on_iteration);
            mcap_writer->writePoses(poses);
            mcap_writer->update();
        }

        if (enable_json_log) {
            const auto pose_graph_info = builder.calculatePoseGraphInfo();
            serialization::writer::writePoseGraphInfoToJSON(json_log_path, pose_graph_info, 0);
        }

        for (size_t i = 1; i <= kOptimizationSteps; i++) {
            poses = builder.optimizePoseGraph();

            if (enable_mcap_log) {
                const Cloud lidar_map_on_iteration =
                    builder.mergeClouds(builder.transformClouds(poses, clouds));

                mcap_writer->writeCloud(lidar_map_on_iteration);
                mcap_writer->writePoses(poses);
                mcap_writer->update();
            }

            if (enable_json_log) {
                const auto pose_graph_info = builder.calculatePoseGraphInfo();
                serialization::writer::writePoseGraphInfoToJSON(json_log_path, pose_graph_info, i);
            }
        }

        clouds = builder.applyGridFilter(clouds);

        const auto lidar_map = builder.mergeClouds(builder.transformClouds(poses, clouds));

        serialization::writer::MCAPWriter::writeCloud(
            mcap_output_folder_path, lidar_map, kOutputTopicLidarMap);
    }
}
