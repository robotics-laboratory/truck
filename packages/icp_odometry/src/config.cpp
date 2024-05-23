#include "icp_odometry/config.h"

#include <yaml-cpp/yaml.h>

#include <iostream>


Config globalConfig;

ConfigLoader::ConfigLoader(const std::string &configFile) {
    std::cout << "ConfigLoader started! " << configFile << "\n";
    try {
        YAML::Node config = YAML::LoadFile(configFile);

        globalConfig.verbose = config["verbose"].as<bool>();
        globalConfig.bagPath = config["bag_path"].as<std::string>();
        globalConfig.referencePointsDistanceThreshold = config["reference_points_distance_threshold"].as<double>();
        globalConfig.icpEdgeMaxDistance = config["icp_edge_max_distance"].as<double>();
        globalConfig.odometryEdgeWeight = config["odometry_edge_weight"].as<double>();
        globalConfig.icpEdgeWeight = config["icp_edge_weight"].as<double>();
        globalConfig.neighborsFilterKnn = config["neighbors_filter_knn"].as<uint32_t>();
        globalConfig.neighborsFilterDistanceThreshold = config["neighbors_filter_distance_threshold"].as<double>();
        globalConfig.enableDebugIcpMatches = config["enable_debug_icp_matches"].as<bool>();
        globalConfig.voxelFilterSize = config["voxel_filter_size"].as<double>();
        globalConfig.resultPath = config["result_path"].as<std::string>();
    } catch (const YAML::Exception &e) {
        std::cerr << "Error loading config file: " << e.what() << std::endl;
    }
}

ConfigLoader global_config_loader("/truck/packages/icp_odometry/config/map_builder.yaml");
