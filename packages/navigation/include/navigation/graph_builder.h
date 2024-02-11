#pragma once

#include "geom/segment.h"
#include "geom/complex_polygon.h"

#include <optional>

namespace truck::navigation::graph {

struct GraphParams {
    enum class Mode : uint8_t { kNearest = 0, searchRadius = 1 } mode;
    size_t k_nearest = 6;
    double search_radius = 2;
};

class GraphBuilder {
  public:
    GraphBuilder(const GraphParams& params);

    GraphBuilder& setNodes(const std::vector<geom::Vec2>& nodes);
    GraphBuilder& setComplexPolygons(const geom::ComplexPolygons& polygons);
    GraphBuilder& build();

    const geom::Segments& getEdges() const;
    const std::vector<geom::Vec2>& getNodes() const;
    const std::vector<std::vector<std::optional<double>>>& getWeights() const;

  private:
    bool collisionFreeEdge(const geom::Segment& edge) const;

    geom::Segments edges_;
    std::vector<geom::Vec2> nodes_;
    std::vector<std::vector<std::optional<double>>> weights_;

    geom::ComplexPolygons polygons_;

    GraphParams params_;
};

}  // namespace truck::navigation::graph