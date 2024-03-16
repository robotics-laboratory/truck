#pragma once

#include "geom/pose.h"
#include "geom/vector.h"

#include <iterator>

namespace truck::geom {

template<typename Container>
class UniformStepper {
  public:
    using IteratorCategory = std::forward_iterator_tag;
    using ContainerIterator = typename Container::const_iterator;

    UniformStepper() = delete;

    UniformStepper(const Container* container) noexcept
        : container_(container)
        , step_length_(1.0)
        , dist_from_milestone_(0.0)
        , milestone_(container_->begin()) {}

    UniformStepper(const Container* container, double step_length) noexcept
        : container_(container)
        , step_length_(step_length)
        , dist_from_milestone_(0.0)
        , milestone_(container_->begin()) {}

    UniformStepper(const Container* container, ContainerIterator milestone) noexcept
        : container_(container)
        , step_length_(1.0)
        , dist_from_milestone_(0.0)
        , milestone_(milestone) {}

    UniformStepper(
        const Container* container, double step_length, ContainerIterator milestone) noexcept
        : container_(container)
        , step_length_(step_length)
        , dist_from_milestone_(0.0)
        , milestone_(milestone) {}

    UniformStepper& SetStepLength(double step_length) noexcept {
        step_length_ = step_length;
        return *this;
    }

    Pose operator*() const noexcept { return GetPose(); }

    UniformStepper& operator++() noexcept {
        double step_left = step_length_;
        while (milestone_ + 1 != container_->end()) {
            double dist_to_next_milestone =
                (*(milestone_ + 1) - *milestone_).len() - dist_from_milestone_;
            if (dist_to_next_milestone > step_left) {
                dist_from_milestone_ += step_left;
                break;
            }
            step_left -= dist_to_next_milestone;
            ++milestone_;
            dist_from_milestone_ = 0.0;
        }
        return *this;
    }

    UniformStepper operator++(int) noexcept {
        UniformStepper it = *this;
        ++it;
        Swap(it);
        return it;
    }

    bool operator==(const UniformStepper<Container>& other) const noexcept {
        return milestone_ == other.milestone_ && dist_from_milestone_ == other.dist_from_milestone_;
    }

    bool operator!=(const UniformStepper<Container>& other) const noexcept {
        return !(*this == other);
    }

  private:
    geom::Pose GetPose() const noexcept {
        geom::AngleVec2 dir =
            (milestone_ + 1 == container_->end()
                 ? AngleVec2::fromVector(*milestone_ - *(milestone_ - 1))
                 : AngleVec2::fromVector(*(milestone_ + 1) - *milestone_));
        return geom::Pose(*milestone_ + dist_from_milestone_ * dir.vec(), dir);
    }

    void Swap(UniformStepper& other) noexcept {
        std::swap(step_length_, other.step_length_);
        std::swap(dist_from_milestone_, other.dist_from_milestone_);
        std::swap(milestone_, other.milestone_);
    }

    const Container* container_ = nullptr;

    double step_length_ = 1.0;

    double dist_from_milestone_ = 0.0;
    ContainerIterator milestone_;
};

}  // namespace truck::geom
