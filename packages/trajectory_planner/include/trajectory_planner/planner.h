#pragma once

#include "trajectory_planner/sampler.h"
#include "trajectory_planner/state.h"

#include "common/array_as_bit.h"
#include "common/math.h"

#include "collision/collision_checker.h"

#include "geom/pose.h"

#include "model/model.h"

#include <random>
#include <vector>

namespace truck::trajectory_planner {

using NodeId = size_t;
using EdgeId = size_t;

struct Node {
    const State* state;
    std::vector<EdgeId> edges;

    double heuristic_cost_from_start;
    double heuristic_cost_to_finish;

    bool is_finish = false;
};

using Nodes = std::vector<Node>;

struct Edge {
    const NodeId to;
    double cost;
    double heuristic_cost;
};

using Edges = std::vector<Edge>;

struct SearchTree {
    NodeId AddNode(Node node) noexcept;

    EdgeId AddEdge(Edge edge) noexcept;

    void Clear() noexcept;

    Nodes nodes;
    std::vector<double> node_probabilities;
    Edges edges;
    NodeId start_node;
    std::vector<NodeId> finish_nodes;
};

class Planner {
  public:
    struct Params {
        size_t batch_size = 100;
        size_t total_batches = 1000;
        size_t total_heuristic_steps = 3;
        double step_resolution = 0.05;
        double min_dist_to_obstacle = 0.1;
        double planning_horizon = 5;
    };

    Planner(const Params& params, const model::Model& model);

    Planner& Build(const StateSpace& state_space) noexcept;

    Planner& SetCollisionChecker(
        std::shared_ptr<const collision::StaticCollisionChecker> collision_checker);

    std::optional<double> HeuristicCost(const State& from, const State& to) const noexcept;

    std::optional<double> Cost(const State& from, const State& to) const noexcept;

    std::optional<double> HeuristicCostFromStart(const State& state) const noexcept;

    std::optional<double> HeuristicCostToFinish(const State& state) const noexcept;

  private:
    bool IsCollisionFree(const geom::Pose& pose) const noexcept;

    bool IsDifferentiallyFeasible(const geom::Poses& motion) const noexcept;

    bool IsKinodynamicFeasible(
        double motion_time, double from_velocity, double to_velocity) const noexcept;

    double AdmissibleMotionTime(
        double motion_length, double from_velocity, double to_velocity) const noexcept;

    Params params_;
    model::Model model_;

    std::shared_ptr<const collision::StaticCollisionChecker> collision_checker_ = nullptr;

    SearchTree search_tree_;
};

}  // namespace truck::trajectory_planner