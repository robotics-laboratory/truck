#include "trajectory_planner/rtree.h"

namespace truck::trajectory_planner {

SpatioTemporalRTree::SpatioTemporalRTree(const Discretization<double>& velocity_discretization)
    : velocity_discretization_(velocity_discretization) {
    layers_.resize(velocity_discretization_.total_states);
}

SpatioTemporalRTree::SpatioTemporalRTree(
    const Discretization<double>& velocity_discretization, const Edge::Estimator& estimator)
    : velocity_discretization_(velocity_discretization), estimator_(estimator) {
    layers_.resize(velocity_discretization_.total_states);
}

SpatioTemporalRTree& SpatioTemporalRTree::Add(Node& node) noexcept {
    size_t velocity_layer = velocity_discretization_(node.state->velocity);
    layers_[velocity_layer].insert({node.state->pose.pos, &node});
    return *this;
}

SpatioTemporalRTree& SpatioTemporalRTree::Remove(Node& node) noexcept {
    size_t velocity_layer = velocity_discretization_(node.state->velocity);
    layers_[velocity_layer].remove({node.state->pose.pos, &node});
    return *this;
}

const std::vector<SpatioTemporalRTree::SearchResult>& SpatioTemporalRTree::RangeSearch(
    const Node& node, double radius) noexcept {
    result_buffer_.clear();
    for (int i = 0; i < velocity_discretization_.total_states; ++i) {
        const double dist_radius =
            radius * (node.state->velocity + velocity_discretization_[i]) * 0.5;
        const RTreeBox range_box(
            geom::Vec2(node.state->pose.pos.x - dist_radius, node.state->pose.pos.y - dist_radius),
            geom::Vec2(node.state->pose.pos.x + dist_radius, node.state->pose.pos.y + dist_radius));
        layers_[i].query(
            bg::index::covered_by(range_box) && bg::index::satisfies([&](const auto& value) {
                return estimator_.HeuristicCost(*node.state, *value.second->state) <= radius;
            }),
            boost::make_function_output_iterator([&](const auto& value) {
                result_buffer_.emplace_back(
                    estimator_.HeuristicCost(*node.state, *value.second->state), value.second);
            }));
    }
    return result_buffer_;
}

SpatioTemporalRTree& SpatioTemporalRTree::Clear() noexcept {
    for (auto& layer : layers_) {
        layer.clear();
    }
    result_buffer_.clear();
    return *this;
}

SpatioTemporalRTree& SpatioTemporalRTree::UpdateEstimator(
    const Edge::Estimator& estimator) noexcept {
    estimator_ = estimator;
    return *this;
}

}  // namespace truck::trajectory_planner