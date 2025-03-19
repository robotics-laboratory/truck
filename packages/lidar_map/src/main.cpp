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
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

namespace po = boost::program_options;

const std::string kInputTopicPointCloud = "/livox/lidar";
const std::string kInputTopicOdom = "/ekf/odometry/filtered";
const std::string kOutputTopicLidarMap = "/lidar_map";
const std::string kOutputTopicPoses = "/poses";
const std::string kOutputTopicClouds = "/clouds";
const std::string kPkgPathLidarMap = ament_index_cpp::get_package_share_directory("lidar_map");

DP toDataPoints(const Cloud& cloud) {
    DP::Labels feature_labels;
    feature_labels.push_back(DataPoints::Label("x", 1));
    feature_labels.push_back(DataPoints::Label("y", 1));
    feature_labels.push_back(DataPoints::Label("z", 1));
    feature_labels.push_back(DataPoints::Label("w", 1));

    DP::Labels descriptor_labels;

    DP data_points(feature_labels, descriptor_labels, cloud.cols());
    data_points.features = cloud;
    return data_points;
}

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
        .icp_config = kPkgPathLidarMap + "/config/icp.yaml",
        .icp_edge_max_dist = 3,
        .odom_edge_weight = 1.0,
        .icp_edge_weight = 3.0,
        .verbose = true};

    Builder builder = Builder(builder_params);

    const size_t optimization_steps = 10;
    const double min_poses_dist = 1;

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
                mcap_input_path, kInputTopicOdom, kInputTopicPointCloud, min_poses_dist);

        poses = toPoses(odom_msgs);
        clouds = toClouds(point_cloud_msgs);

        const auto rotate_poses_by_angle_PI = [](auto& poses) {
            for (auto& pose : poses) {
                pose.dir += truck::geom::Angle::fromRadians(M_PI);
            }
        };

        // TODO (apmilko): fix clouds orientation
        rotate_poses_by_angle_PI(poses);
        clouds = builder.transformClouds(poses, clouds);
        // DP cloud1 = toDataPoints(clouds[266]);
        // DP cloud2 = toDataPoints(clouds[267]);
        // std::ifstream icp_config_stream("/root/truck/packages/lidar_map/config/icp.yaml");
        // PM::ICP icp;
        // icp.loadFromYaml(icp_config_stream);
        // PM::TransformationParameters T = icp(cloud1, cloud2);

        // DP cloud3(cloud1);
        // icp.transformations.apply(cloud3, T);
        // DP cloud3 = toDataPoints(clouds[268]);
        // serialization::writer::MCAPWriter::writeClouds(mcap_output_folder_path, cloud3.features, cloud1.features, cloud2.features);
        serialization::writer::MCAPWriter::writeClouds2(mcap_output_folder_path, std::vector<Cloud>(clouds.begin() + 293, clouds.begin() + 308));
    }
    
    // 2. Construct and optimize pose graph
    {
        // poses = std::vector<decltype(poses)::value_type>(poses.begin(), poses.begin() + 100);
        // clouds = std::vector<Cloud>(clouds.begin(), clouds.begin() + 100);
        // Poses poses2 = std::vector<decltype(poses)::value_type>(poses.begin() + 1050, poses.begin() + 1082);
        // Clouds clouds2 = std::vector<Cloud>(clouds.begin() + 1050, clouds.begin() + 1082);
        // poses = poses1;
        // poses.insert(poses.end(), poses2.begin(), poses2.end());

        // clouds = clouds1;
        // clouds.insert(clouds.end(), clouds2.begin(), clouds2.end());
        // clouds = builder.applyBoundingBoxFilter(clouds, 10.0);
        
        // builder.initPoseGraph(poses, clouds);
        // if (enable_mcap_log) {
        //     const Cloud lidar_map_on_iteration =
        //         builder.mergeClouds(builder.transformClouds(poses, clouds));
        //     Cloud lidar_map_on_iteration_f = builder.applyGridFilter(lidar_map_on_iteration, 0.15);
        //     mcap_writer->writeCloud(lidar_map_on_iteration_f);
        //     mcap_writer->writePoses(poses);
        //     mcap_writer->update();
        // }   

        // if (enable_json_log) {
        //     const auto pose_graph_info = builder.calculatePoseGraphInfo();
        //     serialization::writer::writePoseGraphInfoToJSON(json_log_path, pose_graph_info, 0);
        // }

        // for (size_t i = 1; i <= optimization_steps; i++) {
        //     poses = builder.optimizePoseGraph();

        //     if (enable_mcap_log) {
        //         const Cloud lidar_map_on_iteration =
        //         builder.mergeClouds(builder.transformClouds(poses, clouds));
        //         Cloud lidar_map_on_iteration_f = builder.applyGridFilter(lidar_map_on_iteration, 0.15);
        //         mcap_writer->writeCloud(lidar_map_on_iteration_f);
        //         mcap_writer->writePoses(poses);
        //         mcap_writer->update();
        //     }

        //     if (enable_json_log) {
        //         const auto pose_graph_info = builder.calculatePoseGraphInfo();
        //         serialization::writer::writePoseGraphInfoToJSON(json_log_path, pose_graph_info, i);
        //     }
        // }
        // // clouds = builder.applyDynamicFilter(poses, clouds, 40.0, 30, 0.5);

        // const auto lidar_map = builder.mergeClouds(builder.transformClouds(poses, clouds));
        // Cloud lidar_map_filtered = builder.applyGridFilter(lidar_map, 0.15);

        // serialization::writer::MCAPWriter::writeCloud(
        //     mcap_output_folder_path, lidar_map_filtered, kOutputTopicLidarMap);
    }
}
