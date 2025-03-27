#pragma once

#include "geom/pose.h"
#include "geom/polyline_index.h"
#include "geom/polyline.h"
#include "geom/complex_polygon.h"

#include <vector>

namespace truck::motion_planner {

using Reference = geom::PolylinePoseIndex;

using MilestoneId = size_t;
using NodeId = size_t;
using EdgeId = size_t;

using MilestoneIds = std::vector<MilestoneId>;
using NodeIds = std::vector<NodeId>;
using EdgeIds = std::vector<EdgeId>;

namespace hull {  // TODO: give it a proper name

using Guide = geom::Pose;
using IndexGuide = std::pair<int, Guide>;
using IndexGuides = std::vector<IndexGuide>;
using Milestones = std::vector<class Milestone>;

class Milestone {
  public:
    Milestone(
        MilestoneId id, const geom::Pose& pose, double left_ledge, double right_ledge) noexcept;
    Guide guide(double offset) const;
    IndexGuides getSpacedOutGuides(double spacing) const;
    std::pair<Guide, Guide> getEndpoints() const;
    geom::Segment toSegment() const;

    MilestoneId id;
    geom::Pose pose;
    double left_ledge;
    double right_ledge;
};

// TODO: separate this mess
struct GraphParams {
    double hull_radius;
    double milestone_spacing;
    double node_spacing;
    double raycast_increment;
    double max_edge_slope;
    double safezone_radius;
};

struct Node {
    NodeId id;
    geom::Pose pose;
    std::vector<EdgeId> out;
    std::vector<EdgeId> in;
    MilestoneId milestone_id;
    int milestone_offset;
};

struct Edge {
    EdgeId id;
    NodeId from, to;
    double weight;
};

using Nodes = std::vector<Node>;
using Edges = std::vector<Edge>;

struct Graph {
    Nodes nodes;
    Edges edges;
};

struct GraphContext {
    Milestones milestones = {};
    std::vector<NodeIds> milestone_nodes = {};
};

// TODO: clean & separate interfaces
struct GraphBuild {
    geom::ComplexPolygon map;
    Reference reference;
    Milestones milestones = {};
    Nodes nodes = {};
    Edges edges = {};
    std::vector<NodeIds> milestone_nodes = {};
};

// TODO: annihilate
struct TrajectoryBuild {
    geom::Polyline path;
    geom::Polyline trajectory;
};

}  // namespace hull

class GraphBuilder {
  public:
    GraphBuilder(const hull::GraphParams& params);

    std::pair<hull::Graph, hull::GraphContext> buildGraph(
        const Reference& reference, const geom::ComplexPolygon& map) const;

  private:
    void makeMilestones(hull::GraphBuild& build) const;
    void makeNodes(hull::GraphBuild& build) const;
    void makeEdges(hull::GraphBuild& build) const;

    hull::GraphParams params_;
};

}  // namespace truck::motion_planner
