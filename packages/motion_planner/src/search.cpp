#include "motion_planner/search.h"
#include "geom/distance.h"

#include <set>

namespace truck::motion_planner::search {

geom::PolylineMotionIndex fitSpline(
    const hull::Graph& graph, const hull::GraphParams& params, const Path& path) {
    // TODO: connect all poses of nodes in the path with bezier3 curves
    // maximum curvature of a curve should not exceed `params.max_curvature`
}

Path findShortestPath(
    const hull::Graph& graph, const std::vector<bool>& node_occupancy, NodeId from_id,
    NodeId to_id) {
    const size_t nodes_count = graph.nodes.size();

    VERIFY(from_id < nodes_count);
    VERIFY(to_id < nodes_count);

    const double max_dist = std::numeric_limits<double>::max();

    auto dist = std::vector<double>(nodes_count, max_dist);
    auto prev = std::vector<NodeId>(nodes_count, nodes_count);

    // struct for easier implementation of dijkstra algorithm
    struct NodeWrap {
        NodeId id;
        double from_dist;

        // queue priority
        bool operator<(const NodeWrap& other) const {
            return std::tie(from_dist, id) < std::tie(other.from_dist, other.id);
        }
    };

    auto buildNodeById = [&](NodeId id) -> NodeWrap {
        return NodeWrap{
            .id = id,
            .from_dist = dist[id],
        };
    };

    auto extractPath = [&]() {
        Path path;
        NodeId cur_id = to_id;

        if (prev[cur_id] == nodes_count) {
            return path;
        }

        path.length = dist[cur_id];

        while (prev[cur_id] != nodes_count) {
            path.trace.emplace_back(cur_id);
            cur_id = prev[cur_id];
        }

        path.trace.emplace_back(cur_id);
        std::reverse(path.trace.begin(), path.trace.end());

        return path;
    };

    std::set<NodeWrap> queue;

    // initialization
    dist[from_id] = 0.0;
    queue.insert(buildNodeById(from_id));

    while (!queue.empty()) {
        NodeWrap cur_node = *queue.begin();

        queue.erase(queue.begin());

        // skip on dead vertices
        if (node_occupancy[cur_node.id] == true) {
            continue;
        }

        // exit condition
        if (cur_node.id == to_id || dist[cur_node.id] == max_dist) {
            break;
        }

        // dijkstra loop
        for (const EdgeId& edge_id : graph.nodes[cur_node.id].out) {
            const hull::Edge& edge = graph.edges[edge_id];
            const NodeId neighbor_id = edge.to;

            // skip on dead vertices
            if (node_occupancy[neighbor_id] == true) {
                continue;
            }

            const double alt_dist = cur_node.from_dist + edge.weight;

            // update queue
            if (alt_dist < dist[neighbor_id]) {
                queue.erase(buildNodeById(neighbor_id));

                dist[neighbor_id] = alt_dist;
                prev[neighbor_id] = cur_node.id;

                queue.emplace(buildNodeById(neighbor_id));
            }
        }
    }

    return extractPath();
}

std::vector<bool> getNodeOccupancy(
    const hull::Graph& graph, const collision::StaticCollisionChecker& checker, double threshold) {
    VERIFY(checker.initialized());

    std::vector<bool> node_occupancy(graph.nodes.size(), false);

    for (const auto& node : graph.nodes) {
        // Check if the distance from the checker to the node's pose is less than the threshold
        if (checker.distance(node.pose) < threshold) {
            // Mark the node as occupied
            node_occupancy[node.id] = true;
        }
    }

    return node_occupancy;
}

}  // namespace truck::motion_planner::search
