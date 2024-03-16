#include <hnswlib/hnswlib.h>

#include "trajectory_planner/state.h"

#include <memory>
#include <vector>

namespace truck::trajectory_planner {

class SpatialHNSW {
  public:
    using SearchResult = std::pair<double, size_t>;

    SpatialHNSW() = default;

    SpatialHNSW(
        size_t capacity, size_t total_neightbours = 30, size_t construction_buffer_size = 100);

    SpatialHNSW& Add(const geom::Pose& pose, size_t index) noexcept;

    void RangeSearch(
        const geom::Pose& pose, double radius,
        std::vector<SearchResult>& result_buffer) const noexcept;

  private:
    std::unique_ptr<hnswlib::HierarchicalNSW<double>> alg_hnsw_ = nullptr;
};

class SpatioTemporalHNSW {
  public:
    using SearchResult = std::pair<double, size_t>;

    SpatioTemporalHNSW(
        const Discretization<double>& velocity_discretization, size_t layer_capacity);

    SpatioTemporalHNSW& Add(const State& state, size_t index) noexcept;

    void RangeSearch(
        const State& state, double radius, std::vector<SearchResult>& result_buffer) const noexcept;

  private:
    Discretization<double> velocity_discretization_;

    std::vector<SpatialHNSW> layers_;
};

}  // namespace truck::trajectory_planner