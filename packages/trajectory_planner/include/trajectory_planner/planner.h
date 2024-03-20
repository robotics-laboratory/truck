#pragma once

#include "trajectory_planner/state.h"

#include "common/math.h"

#include "geom/pose.h"
#include "geom/polyline.h"

#include <unordered_set>
#include <vector>

namespace truck::trajectory_planner {

struct Edge;

struct Node {
    const State* state;
    std::vector<const Edge*> edges;
};

struct Edge {
    const Node* to;
    double cost;
    double heuristic_cost;
};

struct SearchTree {
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    Node* start_node;
    std::unordered_set<Node*> finish_nodes;
};

class Planner {
  public:
    struct Params {
        size_t batch_size = 100;
        size_t total_batches = 1000;
    };

    Planner(const Params& params);

    Planner& Build(const StateSpace& state_space) noexcept;

  private:
    Params params_;

    SearchTree search_tree_;
};

}  // namespace truck::trajectory_planner