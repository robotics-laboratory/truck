#pragma once

#include "common/array_as_bit.h"

namespace truck::trajectory_planner {

class Sampler {
  public:
    Sampler(double* probabilities, size_t size) : bit_(probabilities, size) {}

  private:
    ArrayAsBinaryIndexedTree<double> bit_;
};

}  // namespace truck::trajectory_planner