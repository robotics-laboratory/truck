#pragma once

#include "lidar_map/common.h"

#include "geom/pose.h"

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

    std::pair<geom::Poses, Clouds> sliceDataByPosesProximity(
        const geom::Poses& poses, const Clouds& clouds, double poses_min_dist) const;

    geom::Poses optimizePoses(const geom::Poses& poses, const Clouds& clouds);

    Clouds transformClouds(
        const geom::Poses& poses, const Clouds& clouds, bool inverse = false) const;

    Cloud mergeClouds(const Clouds& clouds) const;

    Clouds applyGridFilter(const Clouds& clouds, double cell_size) const;

    Clouds applyDynamicFilter(
        const geom::Poses& poses, const Clouds& clouds_base, double clouds_search_rad,
        size_t min_sim_points_count, double max_sim_points_dist) const;

  private:
    ICP icp_;

    BuilderParams params_;
};

}  // namespace truck::lidar_map
