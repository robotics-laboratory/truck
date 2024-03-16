#include "trajectory_planner/hnsw.h"

#include "geom/pose.h"

#include "common/exception.h"

namespace {

constexpr size_t motion_disceretization = 3;

double StatesDist(const void* from_pose_ptr, const void* to_pose_ptr, const void* qty_ptr) {
    const truck::geom::Pose& from_pose = *static_cast<const truck::geom::Pose*>(from_pose_ptr);
    const truck::geom::Pose& to_pose = *static_cast<const truck::geom::Pose*>(to_pose_ptr);
    return truck::trajectory_planner::MotionLength(
        truck::trajectory_planner::FindMotion(from_pose, to_pose, motion_disceretization));
}

class StateSpace final : public hnswlib::SpaceInterface<double> {
  public:
    StateSpace()
        : fstdistfunc_(StatesDist), dim_(1), data_size_(dim_ * sizeof(truck::geom::Pose)) {}

    size_t get_data_size() final override { return data_size_; }

    hnswlib::DISTFUNC<double> get_dist_func() final override { return fstdistfunc_; }

    void* get_dist_func_param() final override { return &dim_; }

  private:
    hnswlib::DISTFUNC<double> fstdistfunc_;
    size_t dim_;
    size_t data_size_;
} state_space;

}  // namespace

namespace truck::trajectory_planner {

SpatialHNSW::SpatialHNSW(size_t capacity, size_t total_neightbours, size_t construction_buffer_size)
    : alg_hnsw_(std::make_unique<hnswlib::HierarchicalNSW<double>>(
          &state_space, capacity, total_neightbours, construction_buffer_size)) {}

SpatialHNSW& SpatialHNSW::Add(const geom::Pose& pose, size_t index) noexcept {
    VERIFY(alg_hnsw_);
    alg_hnsw_->addPoint(&pose, index);
    return *this;
}

void SpatialHNSW::RangeSearch(
    const geom::Pose& pose, double radius,
    std::vector<SearchResult>& result_buffer) const noexcept {
    VERIFY(alg_hnsw_);
    hnswlib::EpsilonSearchStopCondition<double> stop_condition(
        radius, alg_hnsw_->getMaxElements(), alg_hnsw_->getMaxElements());
    auto result = alg_hnsw_->searchStopConditionClosest(&pose, stop_condition);
    result_buffer.insert(result_buffer.end(), result.begin(), result.end());
}

SpatioTemporalHNSW::SpatioTemporalHNSW(
    const Discretization<double>& velocity_discretization, size_t layer_capacity)
    : velocity_discretization_(velocity_discretization) {
    layers_.reserve(velocity_discretization_.total_states);
    for (size_t i = 0; i < velocity_discretization_.total_states; ++i) {
        layers_.emplace_back(layer_capacity);
    }
}

SpatioTemporalHNSW& SpatioTemporalHNSW::Add(const State& state, size_t index) noexcept {
    size_t velocity_layer = velocity_discretization_(state.velocity);
    layers_[velocity_layer].Add(state.pose, index);
    return *this;
}

void SpatioTemporalHNSW::RangeSearch(
    const State& state, double radius, std::vector<SearchResult>& result_buffer) const noexcept {
    for (size_t i = 0; i < velocity_discretization_.total_states; ++i) {
        const double dist_radius = radius * (state.velocity + velocity_discretization_[i]) * 0.5;
        layers_[i].RangeSearch(state.pose, dist_radius, result_buffer);
    }
}

}  // namespace truck::trajectory_planner
