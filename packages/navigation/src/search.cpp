#include "navigation/search.h"

namespace truck::navigation::search {

geom::Segments toSegments(const std::vector<size_t>& indices, const std::vector<geom::Vec2>& mesh) {
    geom::Segments segments;

    if (indices.size() == 0) {
        return segments;
    }

    for (size_t i = 0; i < indices.size() - 1; i++) {
        segments.emplace_back(geom::Segment(mesh[indices[i]], mesh[indices[i + 1]]));
    }

    return segments;
}

std::vector<geom::Vec2> toPoints(const std::vector<size_t>& indices, const std::vector<geom::Vec2>& mesh) {
    std::vector<geom::Vec2> points;

    for (size_t i = 0; i < indices.size(); i++) {
        points.emplace_back(mesh[indices[i]]);
    }

    return points;
}

std::vector<size_t> DijkstraShortestRoute(
    size_t cur_node, const std::vector<std::optional<size_t>>& prev) {
    std::vector<size_t> route_nodes;

    while (prev[cur_node].has_value()) {
        route_nodes.emplace_back(cur_node);
        cur_node = prev[cur_node].value();
    }

    route_nodes.emplace_back(cur_node);
    std::reverse(route_nodes.begin(), route_nodes.end());

    return route_nodes;
}

std::vector<size_t> Dijkstra(
    const std::vector<std::vector<std::optional<double>>>& weights,
    size_t from_node, size_t to_node) {
    const size_t nodes_count = weights.size();
    const double max_dist = std::numeric_limits<double>::infinity();

    VERIFY(from_node < nodes_count && to_node < nodes_count);

    auto dist = std::vector<double>(nodes_count, max_dist);
    auto prev = std::vector<std::optional<size_t>>(nodes_count, std::nullopt);
    std::set<std::pair<double, size_t>> queue;

    dist[from_node] = 0.0;
    queue.insert(std::make_pair(dist[from_node], from_node));

    while (!queue.empty()) {
        double cur_dist = queue.begin()->first;
        size_t cur_node = queue.begin()->second;

        queue.erase(queue.begin());

        if (cur_node == to_node || dist[cur_node] == max_dist) {
            break;
        }

        for (size_t neighbor_node = 0; neighbor_node < nodes_count; neighbor_node++) {
            auto edge = weights[cur_node][neighbor_node];

            if (edge.has_value()) {
                double alt_dist = cur_dist + edge.value();

                if (alt_dist < dist[neighbor_node]) {
                    queue.erase(std::make_pair(dist[neighbor_node], neighbor_node));

                    dist[neighbor_node] = alt_dist;
                    prev[neighbor_node] = cur_node;

                    queue.insert(std::make_pair(dist[neighbor_node], neighbor_node));
                }
            }
        }
    }
    
    return DijkstraShortestRoute(to_node, prev);
}

}  // namespace truck::navigation::search