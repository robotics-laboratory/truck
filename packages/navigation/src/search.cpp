#include "navigation/search.h"

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

    std::set<std::pair<double, graph::NodeId>> queue;

    auto extract_path = [&]() {
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

    dist[from_id] = 0.0;
    queue.emplace(dist[from_id], from_id);

    while (!queue.empty()) {
        const auto [cur_dist, cur_id] = *queue.begin();

        queue.erase(queue.begin());

        if (cur_id == to_id || dist[cur_id] == max_dist) {
            break;
        }

        for (const graph::EdgeId& edge_id : graph.nodes[cur_id].edges) {
            const graph::Edge& edge = graph.edges[edge_id];
            const graph::NodeId neighbor_id = edge.to;

            const double alt_dist = cur_dist + edge.weight;

            if (alt_dist < dist[neighbor_id]) {
                queue.erase(std::make_pair(dist[neighbor_id], neighbor_id));

                dist[neighbor_id] = alt_dist;
                prev[neighbor_id] = cur_id;

                queue.emplace(dist[neighbor_id], neighbor_id);
            }
        }
    }

    return extract_path();
}

}  // namespace truck::navigation::search
