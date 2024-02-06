#pragma once

#include "geom/segment.h"
#include "common/exception.h"

#include <set>
#include <vector>
#include <optional>

namespace truck::navigation::search {

geom::Segments toSegments(
    const std::vector<size_t>& indices, const std::vector<geom::Vec2>& mesh);

std::vector<geom::Vec2> toPoints(
    const std::vector<size_t>& indices, const std::vector<geom::Vec2>& mesh);

std::vector<size_t> DijkstraShortestRoute(
    size_t cur_node, const std::vector<std::optional<size_t>>& prev);

std::vector<size_t> Dijkstra(
    const std::vector<std::vector<std::optional<double>>>& weights,
    size_t from_node, size_t to_node);

}  // namespace truck::navigation::search