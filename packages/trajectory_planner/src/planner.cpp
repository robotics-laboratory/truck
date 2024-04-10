#include "trajectory_planner/planner.h"

#include "common/math.h"
#include "common/exception.h"

#include <algorithm>

namespace truck::trajectory_planner {

Planner::Planner(const Params& params, const model::Model& model)
    : params_(params)
    , model_(model)
    , truck_state_(params_.truck_state_params)
    , state_space_holder_(params_.state_space_params)
    , tree_holder_(params_.tree_params, params_.state_space_params.Size() + 1, params_.max_edges)
    , sampler_(tree_holder_.nodes_holder.nodes.capacity)
    , verticies_{.rtree = SpatioTemporalRTree(params_.state_space_params.velocity)}
    , samples_{.rtree = SpatioTemporalRTree(params_.state_space_params.velocity)} {}

Planner& Planner::SetCollisionChecker(
    std::shared_ptr<const collision::StaticCollisionChecker> collision_checker) {
    collision_checker_ = std::move(collision_checker);
    return *this;
}

Planner& Planner::Build(
    const State& ego_state, const StateArea& finish_area, const geom::Polyline& route) noexcept {
    Clear();

    if (collision_checker_ && collision_checker_->initialized()) {
        truck_state_.collision_checker = collision_checker_.get();
    }
    truck_state_.model = &model_;

    auto& state_space = state_space_holder_.state_space;
    state_space.truck_state = truck_state_;

    state_space.Build(ego_state, finish_area, route);

    auto& tree = tree_holder_.tree;
    tree.Build(state_space);

    sampler_.Build(tree.nodes);

    verticies_.rtree.UpdateEstimator(*tree.edges.estimator);
    samples_.rtree.UpdateEstimator(*tree.edges.estimator);

    for (int i = 0; i < tree.nodes.size; ++i) {
        auto& node = tree.nodes.data[i];
        switch (node.type) {
            case Node::Type::START:
                node.cost_to_come = 0.0;
                nodes_queue_.emplace(node.cost_to_come + node.heuristic_cost_to_finish, &node);
                verticies_.Insert(node);
                break;
            case Node::Type::FINISH:
                samples_.Insert(node);
                break;
            default:
                break;
        }
    }

    int last_cost_to_finish = CurrentCostToFinish();
    while (current_batch_ < params_.max_batches && tree.edges.size < params_.max_edges) {
        if (nodes_queue_.empty() && edges_queue_.empty()) {
            ++current_batch_;
            int current_cost_to_finish = CurrentCostToFinish();
            if (current_cost_to_finish < last_cost_to_finish) {
                Prune(current_cost_to_finish);
                last_cost_to_finish = current_cost_to_finish;
            }

            int sampled = 0;
            while (!sampler_.Empty() && sampled < params_.batch_size) {
                auto& node = sampler_.Sample();
                sampler_.Remove(node);
                if (node.heuristic_cost_from_start + node.heuristic_cost_from_start >
                    CurrentCostToFinish()) {
                    continue;
                }
                samples_.Insert(node);
                ++sampled;
            }
            // TODO: переписать на кучу с построением O(n)
            for (auto& [node, gen] : verticies_.generation) {
                nodes_queue_.emplace(node->cost_to_come + node->heuristic_cost_to_finish, node);
            }
        }
        while (!nodes_queue_.empty() &&
               (edges_queue_.empty() || nodes_queue_.top().first <= edges_queue_.top().first)) {
            auto& [cost, node] = nodes_queue_.top();
            nodes_queue_.pop();
            if (cost > node->cost_to_come + node->heuristic_cost_to_finish) {
                continue;
            }
            const auto& near_samples = samples_.rtree.RangeSearch(
                *node, Radius(verticies_.generation.size() + samples_.data.size()));
            for (auto& sample : near_samples) {
                if (node->heuristic_cost_from_start +
                        tree.edges.estimator->HeuristicCost(*node->state, *sample.second->state) +
                        sample.second->heuristic_cost_to_finish <
                    CurrentCostToFinish()) {
                    edges_queue_.emplace(
                        node->cost_to_come +
                            tree.edges.estimator->HeuristicCost(
                                *node->state, *sample.second->state) +
                            sample.second->heuristic_cost_to_finish,
                        std::make_pair(node, sample.second));
                }
            }
            auto it = verticies_.generation.find(node);
            if (it != verticies_.generation.end() && it->second == current_batch_) {
                const auto& near_verticies = verticies_.rtree.RangeSearch(
                    *node, Radius(verticies_.generation.size() + samples_.data.size()));
                for (auto& vertex : near_verticies) {
                    if (node->heuristic_cost_from_start + tree.edges.estimator->HeuristicCost(
                                                              *node->state, *vertex.second->state) <
                            CurrentCostToFinish() &&
                        node->cost_to_come + tree.edges.estimator->HeuristicCost(
                                                 *node->state, *vertex.second->state) <
                            vertex.second->cost_to_come) {
                        edges_queue_.emplace(
                            node->cost_to_come +
                                tree.edges.estimator->HeuristicCost(
                                    *node->state, *vertex.second->state) +
                                vertex.second->heuristic_cost_to_finish,
                            std::make_pair(node, vertex.second));
                    }
                }
            }
        }
        if (edges_queue_.empty()) {
            continue;
        }
        auto potential_edge = edges_queue_.top();
        edges_queue_.pop();
        while (potential_edge.first >
               potential_edge.second.first->cost_to_come +
                   tree.edges.estimator->HeuristicCost(
                       *potential_edge.second.first->state, *potential_edge.second.second->state) +
                   potential_edge.second.second->heuristic_cost_to_finish) {
            potential_edge = edges_queue_.top();
            edges_queue_.pop();
        }
        auto& [from, to] = potential_edge.second;
        if (from->cost_to_come + tree.edges.estimator->HeuristicCost(*from->state, *to->state) +
                to->heuristic_cost_to_finish >=
            CurrentCostToFinish()) {
            while (!nodes_queue_.empty()) {
                nodes_queue_.pop();
            }
            while (!edges_queue_.empty()) {
                edges_queue_.pop();
            }
            continue;
        }
        auto exact_edge_cost = tree.edges.estimator->Cost(*from->state, *to->state);
        if (from->cost_to_come + exact_edge_cost + to->heuristic_cost_to_finish <
                CurrentCostToFinish() &&
            from->cost_to_come + exact_edge_cost < to->cost_to_come) {
            if (!verticies_.generation.contains(to)) {
                samples_.Remove(*to);
                verticies_.Insert(*to, current_batch_);

                nodes_queue_.emplace(
                    from->cost_to_come + exact_edge_cost + to->heuristic_cost_to_finish, to);
            }
            tree.edges.AddEdge(*from, *to);
            if (to->type == Node::Type::FINISH && to->cost_to_come < CurrentCostToFinish()) {
                finish_node_ = to;
            }
        }
    }

    if (finish_node_ != nullptr) {
        plan_.push_back(finish_node_);
        while (plan_.back()->type != Node::Type::START) {
            plan_.push_back(plan_.back()->parent_node);
        }
        std::reverse(plan_.begin(), plan_.end());
    }

    return *this;
}

const Node* Planner::GetFinishNode() noexcept { return finish_node_; }

const Plan& Planner::GetPlan() noexcept { return plan_; }

void Planner::Clear() noexcept {
    state_space_holder_.state_space.Clear();
    tree_holder_.tree.Clear();
    verticies_.Clear();
    samples_.Clear();

    while (!nodes_queue_.empty()) {
        nodes_queue_.pop();
    }

    while (!edges_queue_.empty()) {
        edges_queue_.pop();
    }

    finish_node_ = nullptr;

    plan_.clear();
}

double Planner::Radius(int n) const noexcept {
    return 2 * params_.radius_multiplier * std::pow(1 + 0.25, 0.25) *
           std::pow(std::log(n) / n, 0.25);
}

void Planner::Prune(double cost) noexcept {
    while (!samples_.data.empty() && !samples_.by_pass_heuristic.empty() &&
           samples_.by_pass_heuristic.top().first >= cost) {
        auto& [cost, node] = samples_.by_pass_heuristic.top();
        samples_.by_pass_heuristic.pop();
        samples_.Remove(*node);
    }
    while (!verticies_.generation.empty() && !verticies_.by_pass_heuristic.empty() &&
           verticies_.by_pass_heuristic.top().first >= cost) {
        auto& [cost, node] = verticies_.by_pass_heuristic.top();
        verticies_.by_pass_heuristic.pop();
        verticies_.Remove(*node);
    }
    while (!verticies_.generation.empty() && !verticies_.by_cost_to_come.empty() &&
           verticies_.by_cost_to_come.top().first >= params_.planning_horizon) {
        auto& [cost, node] = verticies_.by_cost_to_come.top();
        verticies_.by_cost_to_come.pop();
        if (verticies_.generation.find(node) == verticies_.generation.end()) {
            continue;
        }
        samples_.Insert(*node);
        verticies_.Remove(*node);
    }
}

double Planner::CurrentCostToFinish() const noexcept {
    return finish_node_ == nullptr ? params_.planning_horizon : finish_node_->cost_to_come;
}

void Planner::Verticies::Insert(Node& node, int gen) noexcept {
    auto it = generation.find(&node);
    if (it != generation.end()) {
        if (it->second < gen) {
            it->second = gen;
        }
        return;
    }
    generation[&node] = gen;
    by_pass_heuristic.emplace(node.heuristic_pass_cost, &node);
    by_cost_to_come.emplace(node.cost_to_come, &node);
    rtree.Add(node);
}

void Planner::Verticies::Remove(Node& node) noexcept {
    auto it = generation.find(&node);
    if (it == generation.end()) {
        return;
    }
    generation.erase(it);
    rtree.Remove(node);
}

void Planner::Verticies::Clear() noexcept {
    generation.clear();
    while (!by_pass_heuristic.empty()) {
        by_pass_heuristic.pop();
    }
    while (!by_cost_to_come.empty()) {
        by_cost_to_come.pop();
    }
    rtree.Clear();
}

void Planner::Samples::Insert(Node& node) noexcept {
    if (data.contains(&node)) {
        return;
    }
    data.insert(&node);
    by_pass_heuristic.emplace(node.heuristic_pass_cost, &node);
    rtree.Add(node);
}

void Planner::Samples::Remove(Node& node) noexcept {
    if (!data.contains(&node)) {
        return;
    }
    data.erase(&node);
    rtree.Remove(node);
}

void Planner::Samples::Clear() noexcept {
    data.clear();
    while (!by_pass_heuristic.empty()) {
        by_pass_heuristic.pop();
    }
    rtree.Clear();
}

}  // namespace truck::trajectory_planner