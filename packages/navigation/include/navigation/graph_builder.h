#pragma once

#include "geom/segment.h"

#include <optional>

namespace truck::navigation::graph {

struct GraphParams {
    struct Neighbor {
        enum class Mode : uint8_t {
            kNearest = 0,
            searchRadius = 1
        } mode;

        double search_radius = 2;
        size_t k_nearest = 6;
    } neighbor;
};

struct GraphBuild {
    geom::Segments edges;

    struct Route {
        bool found = false;
        std::vector<geom::Vec2> points;
    } route;
};

class GraphBuilder {
  public:
    GraphBuilder(const GraphParams& params);

    GraphBuild build(const std::vector<geom::Vec2>& mesh);

  private:
    void buildEdges(GraphBuild& graph_build, const std::vector<geom::Vec2>& mesh);
    void buildRoute();

    std::vector<std::vector<std::optional<double>>> weights_;

    GraphParams params_;
};

}  // namespace truck::navigation::graph