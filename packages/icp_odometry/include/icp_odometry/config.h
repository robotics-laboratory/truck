#ifndef TRUCK_CONFIG_H
#define TRUCK_CONFIG_H

#include <string>

struct Config {
    bool verbose;
    std::string bagPath;
    double referencePointsDistanceThreshold;
    double icpEdgeMaxDistance;
    double odometryEdgeWeight;
    double icpEdgeWeight;
    uint32_t neighborsFilterKnn;
    double neighborsFilterDistanceThreshold;
    bool enableDebugIcpMatches;
    double voxelFilterSize;
    std::string resultPath;
};

extern Config globalConfig;

class ConfigLoader {
public:
    ConfigLoader(const std::string &);
};

extern ConfigLoader global_config_loader;

#endif
