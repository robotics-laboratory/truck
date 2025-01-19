#include "cmath"
#include "geom/distance.h"
#include "geom/msg.h"
#include "lidar_map/builder.h"
#include "lidar_map/conversion.h"
#include "lidar_map/serialization.h"
#include "map/map.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/program_options.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <numeric>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace truck::lidar_map;
using namespace truck::map;
using namespace truck::geom;

namespace po = boost::program_options;

const std::string kTopicLaserScan = "/livox/lidar";
const std::string kTopicOdom = "/ekf/odometry/filtered";
const std::string kPkgPathLidarMap = ament_index_cpp::get_package_share_directory("lidar_map");

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
            "mcap-input,mi",
            po::value<std::string>(&mcap_input_path)->required(),
            "path to .mcap file with ride bag")(
            "mcap-output,mo",
            po::value<std::string>(&mcap_output_folder_path)->required(),
            "path to folder where to store .mcap file with 2D LiDAR map (folder shouldn't exist)")(
            "mcap-log,ml",
            po::value<std::string>(&mcap_log_folder_path)->default_value(""),
            "path to folder for mcap logs")(
            "json-log,jl",
            po::value<std::string>(&json_log_path)->default_value(""),
            "path to json log file");

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

    {
        const BuilderParams builder_params{
            .icp_config = kPkgPathLidarMap + "/config/icp.yaml",
            .icp_edge_max_dist = 3,
            .odom_edge_weight = 1.0,
            .icp_edge_weight = 3.0,
            .verbose = true};

        Builder builder = Builder(builder_params);

        BagWriter bag_writer = BagWriter(mcap_log_folder_path, "world", 0.5);

        Poses poses;
        Clouds clouds;

        // 1. Read data from input bag
        {
            const auto odom_msgs = loadOdomTopic(mcap_input_path, kTopicOdom);
            const auto laser_scan_msgs = loadLaserScanTopic(mcap_input_path, kTopicLaserScan);

            const auto [synced_odom_msgs, synced_laser_scan_msgs] =
                syncOdomWithCloud(odom_msgs, laser_scan_msgs);

            const auto all_poses = toPoses(synced_odom_msgs);
            const auto all_clouds = toClouds(synced_laser_scan_msgs);

            std::tie(poses, clouds) = builder.sliceDataByPosesProximity(all_poses, all_clouds, 8.0);

            const auto rotate_poses_by_angle_PI = [](auto& poses) {
                for (auto& pose : poses) {
                    pose.dir += truck::geom::Angle::fromRadians(M_PI);
                }
            };
            rotate_poses_by_angle_PI(poses);  // temporary fix
        }

        // 2. Construct and optimize pose graph
        {
            builder.initPoseGraph(poses, clouds);

            if (enable_mcap_log) {
                const auto tf_merged_clouds =
                    builder.mergeClouds(builder.transformClouds(poses, clouds));
                bag_writer.addOptimizationStep(
                    poses, "/opt/poses", tf_merged_clouds, "/opt/clouds");
            }

            const size_t optimization_steps = 10;

            if (enable_mcap_log) {
                const auto tf_merged_clouds =
                    builder.mergeClouds(builder.transformClouds(poses, clouds));
                bag_writer.addOptimizationStep(
                    poses, "/opt/poses", tf_merged_clouds, "/opt/clouds");
            }

            if (enable_json_log) {
                const auto pose_graph_info = builder.calculatePoseGraphInfo();
                writePoseGraphInfoToJSON(json_log_path, pose_graph_info, 0);
            }

            for (size_t i = 0; i < optimization_steps; i++) {
                poses = builder.optimizePoseGraph(1);

                if (enable_mcap_log) {
                    const auto tf_merged_clouds =
                        builder.mergeClouds(builder.transformClouds(poses, clouds));
                    bag_writer.addOptimizationStep(
                        poses, "/opt/poses", tf_merged_clouds, "/opt/clouds");
            }
                
                if (enable_json_log) {
                    const auto pose_graph_info = builder.calculatePoseGraphInfo();
                    writePoseGraphInfoToJSON(json_log_path, pose_graph_info, i + 1);
                }
            }

            clouds = builder.applyGridFilter(clouds, 0.08);

            const auto lidar_map = builder.mergeClouds(builder.transformClouds(poses, clouds));

            BagWriter::writeCloud(mcap_output_folder_path, lidar_map, "world", "/map/lidar");
        }
    }
}
