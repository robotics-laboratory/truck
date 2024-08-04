#include "navigation/search.h"
#include "geom/distance.h"

#include <set>

namespace truck::navigation::search {

geom::Polyline toPolyline(const graph::Graph& graph, const Path& path) {
    geom::Polyline polyline;

    for (const graph::NodeId node_id : path.trace) {
        polyline.emplace_back(graph.nodes[node_id].point);
    }

    return polyline;
}

Path findShortestPath(const graph::Graph& graph, graph::NodeId from_id, graph::NodeId to_id) {
    const size_t nodes_count = graph.nodes.size();

    VERIFY(from_id < nodes_count);
    VERIFY(to_id < nodes_count);

    const double max_dist = std::numeric_limits<double>::max();

    auto dist = std::vector<double>(nodes_count, max_dist);
    auto prev = std::vector<graph::NodeId>(nodes_count, nodes_count);

    struct NodeWrap {
        graph::NodeId id;
        double from_dist;
        double to_dist;

        // A* heuristic
        bool operator<(const NodeWrap& other) const {
            return from_dist + to_dist < other.from_dist + other.to_dist;
        }
    };

    auto buildNodeById = [&](graph::NodeId id) -> NodeWrap {
        return NodeWrap{
            .id = id,
            .from_dist = dist[id],
            .to_dist = geom::distance(graph.nodes[id].point, graph.nodes[to_id].point)};
    };

    auto extractPath = [&]() {
        Path path;
        graph::NodeId cur_id = to_id;

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

    dist[from_id] = 0.0;
    queue.insert(buildNodeById(from_id));

    while (!queue.empty()) {
        NodeWrap cur_node = *queue.begin();

        queue.erase(queue.begin());

        if (cur_node.id == to_id || dist[cur_node.id] == max_dist) {
            break;
        }

        for (const graph::EdgeId& edge_id : graph.nodes[cur_node.id].edges) {
            const graph::Edge& edge = graph.edges[edge_id];
            const graph::NodeId neighbor_id = edge.to;

            const double alt_dist = cur_node.from_dist + edge.weight;

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

}  // namespace truck::navigation::search
