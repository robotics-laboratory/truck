#include "trajectory_planner/rtree.h"

namespace truck::trajectory_planner {

SpatioTemporalRTree::SpatioTemporalRTree(const Discretization<double>& velocity_discretization)
    : velocity_discretization_(velocity_discretization) {
    layers_.resize(velocity_discretization_.total_states);
}

SpatioTemporalRTree& SpatioTemporalRTree::Add(const State& state) noexcept {
    size_t velocity_layer = velocity_discretization_(state.velocity);
    layers_[velocity_layer].insert({state.pose.pos, &state});
    return *this;
}

void SpatioTemporalRTree::RangeSearch(
    const State& state, double radius, std::vector<SearchResult>& result_buffer) const noexcept {
    for (size_t i = 0; i < velocity_discretization_.total_states; ++i) {
        const double dist_radius = radius * (state.velocity + velocity_discretization_[i]) * 0.5;
        const TreeBox range_box(
            geom::Vec2(state.pose.pos.x - dist_radius, state.pose.pos.y - dist_radius),
            geom::Vec2(state.pose.pos.x + dist_radius, state.pose.pos.y + dist_radius));
        layers_[i].query(
            bg::index::covered_by(range_box) &&
                bg::index::satisfies([&state, &radius](const auto& value) {
                    return MotionTime(
                               MotionLength(FindMotion(state.pose, value.second->pose, 3)),
                               state.velocity,
                               value.second->velocity) <= radius;
                }),
            boost::make_function_output_iterator([&state, &result_buffer](const auto& value) {
                result_buffer.emplace_back(
                    MotionTime(
                        MotionLength(FindMotion(state.pose, value.second->pose, 3)),
                        state.velocity,
                        value.second->velocity),
                    value.second);
            }));
    }
}

}  // namespace truck::trajectory_planner