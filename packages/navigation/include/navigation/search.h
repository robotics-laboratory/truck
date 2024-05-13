#pragma once

#include "navigation/graph_builder.h"

#include "geom/polyline.h"
#include "common/exception.h"

#include <set>
#include <optional>

namespace truck::navigation::search {

struct Path {
    double length;
    std::vector<graph::NodeId> trace;
};

geom::Polyline toPolyline(const graph::Graph& graph, const Path& path);

Path findShortestPath(const graph::Graph& graph, graph::NodeId from, graph::NodeId to);

}  // namespace truck::navigation::search
