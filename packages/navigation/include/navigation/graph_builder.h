#pragma once

#include "geom/complex_polygon.h"

namespace truck::navigation::graph {

struct GraphParams {
    enum class Mode : uint8_t { kNearest = 0, searchRadius = 1 } mode;
    size_t k_nearest = 6;
    double search_radius = 2;
};

using NodeId = size_t;
using EdgeId = size_t;

struct Node {
    NodeId id;
    geom::Vec2 point;
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

class GraphBuilder {
  public:
    GraphBuilder(const GraphParams& params);

    Graph build(const std::vector<geom::Vec2>& mesh, const geom::ComplexPolygons& polygons) const;

  private:
    GraphParams params_;
};

}  // namespace truck::navigation::graph
