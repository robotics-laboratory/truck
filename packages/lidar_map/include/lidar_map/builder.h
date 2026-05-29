#pragma once

#include "geom/pose.h"
#include "lidar_map/common.h"
#include "geom/bounding_box.h"
#include "geom/segment.h"
#include "geom/vector.h"
#include "geom/boost/box.h"

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/hyper_graph.h>

namespace truck::lidar_map {

namespace bgi = boost::geometry::index;

class EdgeData : public g2o::HyperGraph::Data {
  public:
    EdgeData(int value) : _value(value) {}

    bool read(std::istream& is) override { return true; }

    bool write(std::ostream& os) const override { return true; }

    int getValue() const { return _value; }

  private:
    int _value;
};

struct EdgeInfo {
    int from_edge;
    int to_edge;
    double error_val;
    std::string type;
};

struct PoseInfo {
    int id;
    geom::Pose pose;
};

using EdgesInfo = std::vector<EdgeInfo>;
using PosesInfo = std::vector<PoseInfo>;
using SegmentValue = std::pair<geom::BoundingBox, geom::Segment>;
using ICPRTree = bgi::rtree<SegmentValue, bgi::quadratic<16>>;

struct PoseGraphInfo {
    EdgesInfo edges;
    PosesInfo poses;
};

struct BuilderParams {
    std::string icp_config;
    double icp_edge_max_dist = 0.6;
    double icp_edge_min_dist = 1.0;
    double odom_edge_weight = 1.0;
    double icp_edge_weight = 3.0;
    double min_poses_dist = 1.0;
    bool verbose = true;
};

class Builder {
  public:
    Builder(const BuilderParams& params);

    std::pair<geom::Poses, Clouds> sliceDataByPosesProximity(
        const geom::Poses& poses, const Clouds& clouds, double poses_min_dist) const;

    geom::BoundingBox computeBoundingBox(const geom::Segment& segment);

    void initPoseGraph(const geom::Poses& poses, const Clouds& clouds);

    geom::Poses optimizePoseGraph(size_t iterations = 1);

    PoseGraphInfo calculatePoseGraphInfo() const;

    Clouds transformClouds(
        const geom::Poses& poses, const Clouds& clouds, bool inverse = false) const;

    static Cloud mergeClouds(const Clouds& clouds);

    Clouds applyGridFilter(const Clouds& clouds, double cell_size = 0.1) const;

    Cloud applyGridFilter(const Cloud& cloud, double cell_size = 0.1) const;

    Clouds applyBoundingBoxFilter(const Clouds& clouds, double value = 10.0) const;

    Clouds applyDynamicFilter(
        const geom::Poses& poses, const Clouds& clouds_base, double clouds_search_rad,
        size_t min_sim_points_count, double max_sim_points_dist) const;

    Eigen::VectorXf calculateWeightsForReadingCloud(
        const Cloud& reading_cloud, const Cloud& reference_cloud);

    Eigen::Matrix3Xf calculateNormalsForReferenceCloud(const Cloud& reference_cloud);

  private:
    ICP icp_;
    g2o::SparseOptimizer optimizer_;
    BuilderParams params_;
    ICPRTree icp_edges_rtree_;
};

}  // namespace truck::lidar_map
