#pragma once

#include "lidar_map/common.h"

#include "geom/pose.h"

namespace truck::lidar_map {

struct BuilderParams {
    struct Optimizer {
        struct EdgeWeight {
            double icp;
            double odom;
        } edge_weight;

        double icp_edge_max_dist;
        size_t steps;
    } optimizer;

    struct Filter {
        struct Grid {
            double cell_size;
        } grid;

        struct KNN {
            double max_dist;
            int min_neighboring_clouds;
            int max_neighboring_clouds;
        } knn;
    } filter;

    double poses_min_dist;
    bool verbose;
};

class Builder {
  public:
    Builder(const BuilderParams& params, const ICP& icp);

    std::pair<geom::Poses, Clouds> filterByPosesProximity(
        const geom::Poses& poses, const Clouds& clouds) const;

    geom::Poses optimizePoses(const geom::Poses& poses, const Clouds& clouds);

    Clouds transformClouds(const geom::Poses& poses, const Clouds& clouds) const;

    Cloud mergeClouds(const Clouds& clouds) const;

    Cloud applyKNNFilter(const Clouds& clouds) const;

    Cloud applyVoxelGridFilter(const Cloud& cloud, double cell_size) const;

  private:
    ICP icp_;

    BuilderParams params_;
};

}  // namespace truck::lidar_map
