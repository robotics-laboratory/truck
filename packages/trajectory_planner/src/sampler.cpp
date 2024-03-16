#include "trajectory_planner/sampler.h"

namespace truck::trajectory_planner {

Sampler::Sampler(double* probabilities, size_t size)
    : random_device_()
    , generator_(random_device_())
    , distribution_(0.0, 1.0)
    , bit_(probabilities, size) {
    bit_.Build();
}

size_t Sampler::Sample() noexcept {
    return bit_.LowerBound(distribution_(generator_) * bit_.Sum(bit_.Size()));
}

void Sampler::Remove(size_t k) noexcept { bit_.Add(k, -bit_.Sum(k, k + 1)); }

}  // namespace truck::trajectory_planner