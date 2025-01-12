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
