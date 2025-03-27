#pragma once

#include "motion_planner/graph_builder.h"

#include "common/exception.h"
#include "geom/motion_state.h"
#include "collision/collision_checker.h"

namespace truck::motion_planner::search {

struct Path {
    double length;
    NodeIds trace;
};

geom::MotionStates fitSpline(const hull::Nodes& nodes, const Path& path);

Path findShortestPath(
    const hull::Graph& graph, const std::vector<bool>& node_occupancy, NodeId from, NodeId to);

std::vector<bool> getNodeOccupancy(
    const hull::Graph& graph, const collision::StaticCollisionChecker& checker);

}  // namespace truck::motion_planner::search
