#pragma once

#include "trajectory_planner/state.h"

#include "collision/collision_checker.h"

#include "geom/bezier.h"
#include "geom/pose.h"
#include "motion/trajectory.h"

#include "model/model.h"

#include <memory>

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

using NodesDataPtr = std::unique_ptr<Node[]>;

class Node::Estimator {
  public:
    Estimator() = default;

    double HeuristicCostFromStart(const State& state) const noexcept;

    double HeuristicCostToFinish(const State& state) const noexcept;

    Node::Estimator& Reset(StateSpace state_space) noexcept;

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

    Nodes& Reset(Node* ptr) noexcept;

    Node* data = nullptr;
    int size = 0;

    Node::Estimator estimator;
};

struct NodesHolder {
    NodesHolder() = default;

    NodesHolder(Nodes&& nodes, NodesDataPtr&& nodes_ptr) noexcept;

    NodesHolder(const NodesHolder&) = delete;

    NodesHolder(NodesHolder&&) = default;

    NodesHolder& operator=(const NodesHolder&) = delete;

    NodesHolder& operator=(NodesHolder&&) & = default;

    ~NodesHolder() = default;

    Nodes nodes;
    NodesDataPtr nodes_ptr = nullptr;
};

NodesHolder MakeNodes(int capacity);

struct Edge {
    class Estimator;

    Node* from = nullptr;
    Node* to = nullptr;

    double heuristic_cost = INF_COST;
};

using EdgesDataPtr = std::unique_ptr<Edge[]>;

class Edge::Estimator {
  public:
    Estimator() = default;

    Estimator(size_t total_heuristic_steps, double step_resolution);

    double HeuristicCost(const Node& from, const Node& to) const noexcept;

    double Cost(const Node& from, const Node& to) const noexcept;

    Edge::Estimator& Reset(StateSpace state_space) noexcept;

  private:
    bool IsDifferentiallyFeasible(const geom::CurvePoses& motion) const noexcept;

    bool IsKinodynamicFeasible(
        const geom::CurvePoses& motion, double motion_time, double from_velocity,
        double to_velocity, bool check_steering = true, double eps = 1e-7) const noexcept;

    size_t total_heuristic_steps_ = 0;
    double step_resolution_ = 0.0;

    StateSpace state_space_;
};

struct Edges {
    Edge& AddEdge(Node& from, Node& to) noexcept;

    Edges& Clear() noexcept;

    Edges& Reset(Edge* ptr) noexcept;

    Edge* data = nullptr;
    int size = 0;

    Edge::Estimator estimator;
};

struct EdgesHolder {
    EdgesHolder() = default;

    EdgesHolder(Edges&& edges, EdgesDataPtr&& edges_ptr) noexcept;

    EdgesHolder(const EdgesHolder&) = delete;

    EdgesHolder(EdgesHolder&&) = default;

    EdgesHolder& operator=(const EdgesHolder&) = delete;

    EdgesHolder& operator=(EdgesHolder&&) & = default;

    ~EdgesHolder() = default;

    Edges edges;
    EdgesDataPtr edges_ptr = nullptr;
};

EdgesHolder MakeEdges(int capacity);

struct Tree {
    struct Params {
        size_t total_heuristic_steps = 3;
        double step_resolution = 0.01;
    };

    Tree& Build(const StateSpace& state_space) noexcept;

    Tree& Clear() noexcept;

    Tree& Reset(Nodes nodes, Edges edges) noexcept;

    Params params;

    Nodes nodes;

    Nodes start_nodes;
    Nodes finish_nodes;
    Nodes regular_nodes;

    Edges edges;
};

struct TreeHolder {
    TreeHolder() = default;

    TreeHolder(Tree&& tree, NodesHolder&& nodes_holder, EdgesHolder&& edges_holder) noexcept;

    TreeHolder(const TreeHolder&) = delete;

    TreeHolder(TreeHolder&&) = default;

    TreeHolder& operator=(const TreeHolder&) = delete;

    TreeHolder& operator=(TreeHolder&&) & = default;

    ~TreeHolder() = default;

    Tree tree;

    NodesHolder nodes_holder;
    EdgesHolder edges_holder;
};

TreeHolder MakeTree(const Tree::Params& params, int nodes_capacity, int edges_capacity);

using Plan = std::vector<Node*>;

motion::Trajectory ToTrajectory(const Plan& plan, double step_resolution = 0.01, double eps = 1e-7);

}  // namespace truck::trajectory_planner