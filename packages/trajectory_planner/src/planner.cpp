#include "trajectory_planner/planner.h"

#include "common/math.h"
#include "common/exception.h"

#include <algorithm>

namespace truck::trajectory_planner {

Planner::Planner(const Params& params)
    : params_(params)
    , truck_state_(params_.truck_state_params)
    , state_space_holder_(MakeStateSpace(params_.state_space_params))
    , tree_holder_(
          MakeTree(params_.tree_params, params_.state_space_params.Size(), params_.max_edges))
    , sampler_(params_.state_space_params.Size())
    , verticies_{.rtree = SpatioTemporalRTree(params_.state_space_params.velocity)}
    , samples_{.rtree = SpatioTemporalRTree(params_.state_space_params.velocity)} {}

Planner& Planner::SetModel(std::shared_ptr<const model::Model> model) {
    model_ = std::move(model);
    return *this;
}

Planner& Planner::SetCollisionChecker(
    std::shared_ptr<const collision::StaticCollisionChecker> collision_checker) {
    collision_checker_ = std::move(collision_checker);
    return *this;
}

Planner& Planner::Build(
    const State& ego_state, const StateArea& finish_area, const geom::Polyline& route) noexcept {
    Clear();

    VERIFY(model_);
    truck_state_.model = model_.get();

    auto& state_space = state_space_holder_.state_space;
    state_space.truck_state = truck_state_;

    state_space.Build(ego_state, finish_area, route);

    auto& tree = tree_holder_.tree;
    auto& edges_estimator = tree.edges.estimator;
    tree.Build(state_space);

    verticies_.rtree.UpdateEstimator(edges_estimator);
    samples_.rtree.UpdateEstimator(edges_estimator);

    for (int i = 0; i < tree.start_nodes.size; ++i) {
        auto& node = tree.start_nodes.data[i];
        node.cost_to_come = 0.0;
        nodes_queue_.emplace(node.cost_to_come + node.heuristic_cost_to_finish, &node);
        verticies_.Insert(node, 0);
    }

    for (int i = 0; i < tree.finish_nodes.size; ++i) {
        auto& node = tree.finish_nodes.data[i];
        samples_.Insert(node);
    }

    sampler_.Build(tree.regular_nodes);

    int last_cost_to_finish = CurrentCostToFinish();
    while (current_batch_ < params_.max_batches && tree.edges.size < params_.max_edges) {
        if (nodes_queue_.empty() && edges_queue_.empty()) {
            if (sampler_.Empty()) {
                break;
            }

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
                ++sampled;
                if (node.heuristic_pass_cost > CurrentCostToFinish()) {
                    continue;
                }
                samples_.Insert(node);
            }
            // TODO: переписать на кучу с построением O(n)
            for (auto& [node, gen] : verticies_.generation) {
                nodes_queue_.emplace(node->cost_to_come + node->heuristic_cost_to_finish, node);
            }
        }

        while (!nodes_queue_.empty() &&
               (edges_queue_.empty() || nodes_queue_.top().first <= edges_queue_.top().first)) {
            auto cost = nodes_queue_.top().first;
            auto& from = *nodes_queue_.top().second;
            nodes_queue_.pop();

            if (cost > from.cost_to_come + from.heuristic_cost_to_finish) {
                continue;
            }

            const auto& near_samples = samples_.rtree.RangeSearch(
                from, Radius(verticies_.generation.size() + samples_.data.size()));
            for (auto& near_sample : near_samples) {
                if (tree.edges.size == params_.max_edges) {
                    break;
                }
                auto& to = *near_sample.second;
                auto& edge = tree.edges.AddEdge(from, to);
                if (from.heuristic_cost_from_start + edge.heuristic_cost +
                        to.heuristic_cost_to_finish <
                    CurrentCostToFinish()) {
                    edges_queue_.emplace(
                        from.cost_to_come + edge.heuristic_cost + to.heuristic_cost_to_finish,
                        &edge);
                }
            }

            auto it = verticies_.generation.find(&from);
            if (it != verticies_.generation.end() && it->second == current_batch_) {
                const auto& near_verticies = verticies_.rtree.RangeSearch(
                    from, Radius(verticies_.generation.size() + samples_.data.size()));
                for (auto& near_vertex : near_verticies) {
                    if (tree.edges.size == params_.max_edges) {
                        break;
                    }
                    auto& to = *near_vertex.second;
                    auto& edge = tree.edges.AddEdge(from, to);
                    if (from.heuristic_cost_from_start + edge.heuristic_cost +
                                to.heuristic_cost_to_finish <
                            CurrentCostToFinish() &&
                        from.cost_to_come + edge.heuristic_cost < to.cost_to_come) {
                        edges_queue_.emplace(
                            from.cost_to_come + edge.heuristic_cost + to.heuristic_cost_to_finish,
                            &edge);
                    }
                }
            }
        }

        if (edges_queue_.empty()) {
            continue;
        }

        auto potential_edge = edges_queue_.top();
        edges_queue_.pop();
        while (!edges_queue_.empty() &&
               potential_edge.first > potential_edge.second->from->cost_to_come +
                                          potential_edge.second->heuristic_cost +
                                          potential_edge.second->to->heuristic_cost_to_finish) {
            potential_edge = edges_queue_.top();
            edges_queue_.pop();
        }

        auto& edge = *potential_edge.second;
        auto& from = *edge.from;
        auto& to = *edge.to;

        if (from.cost_to_come + edge.heuristic_cost + to.heuristic_cost_to_finish >=
            CurrentCostToFinish()) {
            while (!nodes_queue_.empty()) {
                nodes_queue_.pop();
            }
            while (!edges_queue_.empty()) {
                edges_queue_.pop();
            }
            continue;
        }

        auto edge_cost = edges_estimator.Cost(from, to);

        if (from.cost_to_come + edge_cost + to.heuristic_cost_to_finish < CurrentCostToFinish() &&
            from.cost_to_come + edge_cost < to.cost_to_come) {
            if (!verticies_.generation.contains(&to)) {
                samples_.Remove(to);
                verticies_.Insert(to, current_batch_);

                nodes_queue_.emplace(
                    from.cost_to_come + edge_cost + to.heuristic_cost_to_finish, &to);
            }
            if (to.cost_to_come > from.cost_to_come + edge_cost) {
                to.cost_to_come = from.cost_to_come + edge_cost;
                to.parent_node = &from;
            }
            if (to.type == Node::Type::FINISH && to.cost_to_come < CurrentCostToFinish()) {
                finish_node_ = &to;
            }
        }
    }

    if (finish_node_ != nullptr) {
        plan_.push_back(finish_node_);
        while (plan_.back()->type != Node::Type::START) {
            plan_.push_back(plan_.back()->parent_node);
        }
        std::reverse(plan_.begin(), plan_.end());

        trajectory_ = ToTrajectory(plan_, params_.tree_params.step_resolution);
    }

    return *this;
}

const Node* Planner::GetFinishNode() const noexcept { return finish_node_; }

const Plan& Planner::GetPlan() const noexcept { return plan_; }

const Nodes& Planner::GetNodes() const noexcept { return tree_holder_.tree.nodes; }

const motion::Trajectory& Planner::GetTrajectory() const noexcept { return trajectory_; }

Planner& Planner::Clear() noexcept {
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
    current_batch_ = 0;

    plan_.clear();
    trajectory_.states.clear();

    return *this;
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