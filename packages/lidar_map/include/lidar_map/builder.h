#pragma once

#include "lidar_map/common.h"

#include "geom/pose.h"

#include <vector>

namespace truck::lidar_map {

struct BuilderParams {
    std::string icp_config;

    double icp_edge_max_dist = 0.6;

    double odom_edge_weight = 1.0;
    double icp_edge_weight = 3.0;

    size_t optimizer_steps = 10;

    bool verbose = true;
};

class Builder {
  public:
    Builder(const BuilderParams& params);

    std::pair<geom::Poses, Clouds> filterByPosesProximity(
        const geom::Poses& poses, const Clouds& clouds, double poses_min_dist) const;

    geom::Poses optimizePoses(const geom::Poses& poses, const Clouds& clouds);

    Clouds transformClouds(const geom::Poses& poses, const Clouds& clouds) const;

    Cloud mergeClouds(const Clouds& clouds) const;

    Cloud mergeCloudsByPointsSimilarity(
        const geom::Poses& poses, const Clouds& clouds, int sim_points_min_count,
        double sim_points_max_dist, int clouds_range = -1) const;

    Clouds applyGridFilter(const Clouds& clouds, double cell_size) const;

  private:
    std::vector<std::vector<size_t>> nearestPosesToEachPose(
        const geom::Poses& poses, double poses_max_dist, uint32_t poses_max_num) const;

    ICP icp_;

    BuilderParams params_;
};

}  // namespace truck::lidar_map
