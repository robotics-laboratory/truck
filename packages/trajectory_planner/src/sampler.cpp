#include "trajectory_planner/sampler.h"

namespace truck::trajectory_planner {

Sampler::Sampler(const double* probabilities, size_t size)
    : random_device_()
    , generator_(random_device_())
    , distribution_(0.0, 1.0)
    , probabilities_(probabilities, probabilities + size) {
    ArrayAsBinaryIndexedTree(probabilities_.data(), probabilities_.size()).Build();
}

size_t Sampler::Sample() noexcept {
    auto bit = ArrayAsBinaryIndexedTree(probabilities_.data(), probabilities_.size());
    return bit.LowerBound(distribution_(generator_) * bit.Sum(bit.Size()));
}

void Sampler::Remove(size_t k) noexcept {
    auto bit = ArrayAsBinaryIndexedTree(probabilities_.data(), probabilities_.size());
    bit.Add(k, -bit.Sum(k, k + 1));
}

}  // namespace truck::trajectory_planner