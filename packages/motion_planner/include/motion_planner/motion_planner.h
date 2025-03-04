#pragma once

#include "geom/motion_state.h"
#include "geom/polyline_index.h"
#include "geom/polyline.h"

#include <vector>

namespace truck::motion_planner {

using Reference = geom::PolylineMotionIndex;

using MilestoneId = size_t;
using NodeId = size_t;
using EdgeId = size_t;

namespace hull {

class Milestone {
  public:
    Milestone(size_t id, const geom::Pose& pose, double left_ledge, double right_ledge) noexcept;
    geom::Pose guide(double offset) const;
    geom::Poses getSpacedOutGuides(double spacing) const;
    std::pair<geom::Pose, geom::Pose> getEndpoints() const;
    geom::Segment toSegment() const;

    MilestoneId id;
    geom::Pose pose;
    double left_ledge;
    double right_ledge;
};

using Milestones = std::vector<Milestone>;

struct GraphParams {
    double hull_radius;
    double milestone_spacing;
    double node_spacing;
};

struct Node {
    NodeId id;
    geom::Pose pose;
    std::vector<EdgeId> edges;
};

struct Edge {
    NodeId from, to;
    double weight;
};

using Nodes = std::vector<Node>;
using Edges = std::vector<Edge>;

struct Graph {
    Nodes nodes;
    Edges edges;
};

struct GraphBuild {
    Reference reference;
    Milestones milestones = {};
    Nodes nodes = {};
    Edges edges = {};
    std::vector<std::size_t> node_milestone_map = {};
};

struct TrajectoryBuild {
    geom::Polyline trajectory;
    geom::Polyline path;
};

}  // namespace hull

class GraphBuilder {
  public:
    GraphBuilder(const hull::GraphParams& params);

    hull::Graph buildGraph(const Reference& reference) const;

  private:
    void makeMilestones(hull::GraphBuild& build) const;
    void makeNodes(hull::GraphBuild& build) const;
    void makeEdges(hull::GraphBuild& build) const;

    hull::GraphParams params_;
};

}  // namespace truck::motion_planner
