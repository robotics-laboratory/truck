#pragma once

#include "trajectory_planner/state.h"

#include "collision/collision_checker.h"

#include "geom/pose.h"

#include "model/model.h"

#include <memory>
#include <optional>

namespace truck::trajectory_planner {

static const double INF_COST = std::numeric_limits<double>::max() * 0.33;

struct Edge;

struct Node {
    class Estimator;

    enum class Type : uint8_t { REGULAR, START, FINISH };

    const State* state = nullptr;

    Node* parent_node = nullptr;

    Type type = Type::REGULAR;

    double heuristic_cost_from_start = INF_COST;
    double heuristic_cost_to_finish = INF_COST;
    double heuristic_pass_cost = INF_COST;

    double probability = 0.0;

    double cost_to_come = INF_COST;
};

class Node::Estimator {
  public:
    Estimator() = default;

    Estimator(const StateSpace& state_space);

    double HeuristicCostFromStart(const State& to) const noexcept;

    double HeuristicCostToFinish(const State& from) const noexcept;

  private:
    double AdmissibleMotionTime(
        double motion_length, double from_velocity, double to_velocity) const noexcept;

    double HeuristicCost(const States& from, const State& to) const noexcept;

    StateSpace state_space_;
};

struct Nodes {
    Node& AddNode(const State& state, Node::Type type) noexcept;

    Nodes& FillProbabilities() noexcept;

    Nodes& Clear() noexcept;

    Node* data = nullptr;
    int size = 0;
    int capacity = 0;

    std::optional<Node::Estimator> estimator = std::nullopt;
};

struct NodesHolder {
    NodesHolder(int capacity);

    NodesHolder(const NodesHolder&) = delete;

    NodesHolder(NodesHolder&&) = default;

    NodesHolder& operator=(const NodesHolder&) = delete;

    NodesHolder& operator=(NodesHolder&&) & = default;

    ~NodesHolder() = default;

    Nodes nodes;
    std::unique_ptr<Node[]> nodes_ptr = nullptr;
};

struct Edge {
    class Estimator;

    const Node* from = nullptr;
    const Node* to = nullptr;

    double heuristic_cost = INF_COST;
    double cost = INF_COST;
};

class Edge::Estimator {
  public:
    Estimator() = default;

    Estimator(
        const StateSpace& state_space, size_t total_heuristic_steps = 3,
        double step_resolution = 0.05);

    double HeuristicCost(const State& from, const State& to) const noexcept;

    double Cost(const State& from, const State& to) const noexcept;

  private:
    bool IsDifferentiallyFeasible(const geom::Poses& motion) const noexcept;

    bool IsKinodynamicFeasible(
        const geom::Poses& motion, double motion_time, double from_velocity, double to_velocity,
        double eps = 1e-7) const noexcept;

    StateSpace state_space_;
    size_t total_heuristic_steps_ = 0;
    double step_resolution_ = 0.0;
};

struct Edges {
    Edge& AddEdge(Node& from, Node& to) noexcept;

    Edges& Clear() noexcept;

    Edge* data = nullptr;
    int size = 0;
    int capacity = 0;

    std::optional<Edge::Estimator> estimator = std::nullopt;
};

struct EdgesHolder {
    EdgesHolder(int capacity);

    EdgesHolder(const EdgesHolder&) = delete;

    EdgesHolder(EdgesHolder&&) = default;

    EdgesHolder& operator=(const EdgesHolder&) = delete;

    EdgesHolder& operator=(EdgesHolder&&) & = default;

    ~EdgesHolder() = default;

    Edges edges;
    std::unique_ptr<Edge[]> edges_ptr = nullptr;
};

struct Tree {
    struct Params {
        size_t total_heuristic_steps = 3;
        double step_resolution = 0.01;
    };

    Tree& Build(const StateSpace& state_space) noexcept;

    void Clear() noexcept;

    Params params;

    Nodes nodes;
    Edges edges;
};

struct TreeHolder {
    TreeHolder(const Tree::Params& params, int nodes_capacity, int edges_capacity);

    TreeHolder(const TreeHolder&) = delete;

    TreeHolder(TreeHolder&&) = default;

    TreeHolder& operator=(const TreeHolder&) = delete;

    TreeHolder& operator=(TreeHolder&&) & = default;

    ~TreeHolder() = default;

    NodesHolder nodes_holder;
    EdgesHolder edges_holder;

    Tree tree;
};

}  // namespace truck::trajectory_planner