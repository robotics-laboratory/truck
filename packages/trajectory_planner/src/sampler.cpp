#include "trajectory_planner/sampler.h"

#include "common/exception.h"

namespace truck::trajectory_planner {

Sampler::Sampler(int capacity)
    : generator_()
    , distribution_(0.0, 1.0)
    , data_(std::make_unique<double[]>(capacity))
    , capacity_(capacity) {}

Sampler& Sampler::Build(const Nodes& nodes) noexcept {
    VERIFY(nodes.size <= capacity_);

    nodes_ = nodes.data;
    size_ = nodes.size;
    for (int i = 0; i < nodes.size; ++i) {
        data_[i] = nodes_[i].probability;
        if (data_[i] < eps) {
            data_[i] = 0.0;
            --size_;
        }
    }

    bit_ = ArrayAsBinaryIndexedTree<double>(data_.get(), capacity_).Build();

    return *this;
}

Node& Sampler::Sample() noexcept {
    Node* node = nodes_ + bit_.LowerBound(distribution_(generator_) * bit_.Sum(bit_.Size()));
    if (node->probability < eps) {
        return *(node + 1);
    }
    return *node;
}

void Sampler::Remove(const Node& node) noexcept {
    auto node_id = &node - nodes_;
    double probability = bit_.Sum(node_id, node_id + 1);
    if (probability < eps) {
        return;
    }
    bit_.Add(node_id, -probability);
    --size_;
}

int Sampler::Size() const noexcept { return size_; }

bool Sampler::Empty() const noexcept { return size_ == 0; }

}  // namespace truck::trajectory_planner