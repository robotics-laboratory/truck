#pragma once

#include "trajectory_planner/tree.h"

#include "common/array_as_bit.h"

#include <random>
#include <memory>

namespace truck::trajectory_planner {

class Sampler {
  public:
    static constexpr double eps = 1e-7;

    Sampler() = default;

    Sampler(int capacity);

    Sampler& Build(const Nodes& nodes) noexcept;

    Node& Sample() noexcept;

    void Remove(const Node& node) noexcept;

    int Size() const noexcept;

    bool Empty() const noexcept;

  private:
    std::mt19937 generator_;
    std::uniform_real_distribution<> distribution_;

    std::unique_ptr<double[]> data_ = nullptr;
    int capacity_ = 0;

    ArrayAsBinaryIndexedTree<double> bit_;

    Node* nodes_ = nullptr;
    int size_ = 0;
};

}  // namespace truck::trajectory_planner