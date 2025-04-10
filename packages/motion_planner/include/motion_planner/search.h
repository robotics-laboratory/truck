#pragma once

#include "motion_planner/graph_builder.h"

#include "common/exception.h"
#include "geom/motion_state.h"
#include "collision/collision_checker.h"

#include <set>
#include <optional>

namespace truck::motion_planner::search {

struct Path {
    double length;
    NodeIds trace;
};

geom::MotionStates fitSpline(const hull::Nodes& nodes, const Path& path);
geom::Polyline toPolyline(const hull::Nodes& nodes, const Path& path);
std::optional<Path> findShortestPath(
    const hull::Graph& graph, const std::vector<bool>& node_occupancy, NodeId from_id,
    const std::set<NodeId>& to_ids);

std::vector<bool> getNodeOccupancy(
    const hull::Nodes& nodes, const collision::StaticCollisionChecker& checker,
    const geom::Transform& tf, double threshold);

}  // namespace truck::motion_planner::search
