#pragma once

#include "trajectory_planner/rtree.h"
#include "trajectory_planner/sampler.h"
#include "trajectory_planner/state.h"
#include "trajectory_planner/tree.h"

#include "common/math.h"

#include "collision/collision_checker.h"

#include "model/model.h"

#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace truck::trajectory_planner {

using Plan = std::vector<Node*>;

class Planner {
  public:
    struct Params {
        TruckState::Params truck_state_params;
        StateSpace::Params state_space_params;
        Tree::Params tree_params;
        double planning_horizon = 100.0;
        int batch_size = 500;
        int max_batches = 300;
        int max_edges = 100000;
        double radius_multiplier = 30;
    };

    Planner(const Params& params, const model::Model& model);

    Planner& Build(
        const State& ego_state, const StateArea& finish_area, const geom::Polyline& route) noexcept;

    Planner& SetCollisionChecker(
        std::shared_ptr<const collision::StaticCollisionChecker> collision_checker);

    const Node* GetFinishNode() noexcept;

    const Plan& GetPlan() noexcept;

    void Clear() noexcept;

  private:
    double Radius(int n) const noexcept;

    void Prune(double cost) noexcept;

    double CurrentCostToFinish() const noexcept;

    Params params_;
    model::Model model_;

    std::shared_ptr<const collision::StaticCollisionChecker> collision_checker_ = nullptr;

    TruckState truck_state_;

    StateSpaceHolder state_space_holder_;
    TreeHolder tree_holder_;
    Sampler sampler_;

    struct Verticies {
        void Insert(Node& node, int gen = 0) noexcept;

        void Remove(Node& node) noexcept;

        void Clear() noexcept;

        std::unordered_map<Node*, int> generation;
        std::priority_queue<std::pair<double, Node*>> by_pass_heuristic;
        std::priority_queue<std::pair<double, Node*>> by_cost_to_come;
        SpatioTemporalRTree rtree;
    } verticies_;

    struct Samples {
        void Insert(Node& node) noexcept;

        void Remove(Node& node) noexcept;

        void Clear() noexcept;

        std::unordered_set<Node*> data;
        std::priority_queue<std::pair<double, Node*>> by_pass_heuristic;
        SpatioTemporalRTree rtree;
    } samples_;

    std::priority_queue<
        std::pair<double, Node*>, std::vector<std::pair<double, Node*>>,
        std::greater<std::pair<double, Node*>>>
        nodes_queue_;
    std::priority_queue<
        std::pair<double, std::pair<Node*, Node*>>,
        std::vector<std::pair<double, std::pair<Node*, Node*>>>,
        std::greater<std::pair<double, std::pair<Node*, Node*>>>>
        edges_queue_;

    Node* finish_node_ = nullptr;
    int current_batch_ = 0;

    Plan plan_;
};

}  // namespace truck::trajectory_planner