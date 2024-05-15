#pragma once

#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace rosaruco {

/*
@brief Dijkstra's Shortest Path Algorithm Implementation
@param get_weight get_weight(i, j) equals to weight of edge between i or j, or inf if there is no
edge
@param distance length of shortest path from start_node
@param last_node last node in shortest path from start_node
*/
void Dijkstra(
    int nodes_count, int start_node, std::vector<double>& distance, std::vector<int>& last_node,
    const std::function<double(int, int)>& get_weight);

}  // namespace rosaruco
