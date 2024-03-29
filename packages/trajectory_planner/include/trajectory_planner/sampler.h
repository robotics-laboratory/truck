#pragma once

#include "common/array_as_bit.h"

#include <random>

namespace truck::trajectory_planner {

class Sampler {
  public:
    Sampler();

    Sampler(std::vector<double> data);

    size_t Sample() noexcept;

    void Remove(size_t k) noexcept;

  private:
    std::random_device random_device_;
    std::mt19937 generator_;
    std::uniform_real_distribution<> distribution_;

    std::vector<double> data_;
    ArrayAsBinaryIndexedTree<double> bit_;
};

}  // namespace truck::trajectory_planner