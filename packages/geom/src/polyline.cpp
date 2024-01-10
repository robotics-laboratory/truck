#include "geom/polyline.h"

namespace truck::geom {

UniformIterator Polyline::ubegin() const noexcept { return UniformIterator(this); }

UniformIterator Polyline::ubegin(double step_length) const noexcept {
    return UniformIterator(this, step_length);
}

UniformIterator Polyline::uend() const noexcept { return UniformIterator(this, this->end() - 1); }

UniformIterator::UniformIterator(const Container* polyline) noexcept
    : polyline_(polyline)
    , step_length_(1.0)
    , dist_from_milestone_(0.0)
    , milestone_(polyline_->begin()) {}

UniformIterator::UniformIterator(const Container* polyline, double step_length) noexcept
    : polyline_(polyline)
    , step_length_(step_length)
    , dist_from_milestone_(0.0)
    , milestone_(polyline_->begin()) {}

UniformIterator::UniformIterator(const Container* polyline, ContainerIterator milestone) noexcept
    : polyline_(polyline), step_length_(1.0), dist_from_milestone_(0.0), milestone_(milestone) {}

UniformIterator::UniformIterator(
    const Container* polyline, double step_length, ContainerIterator milestone) noexcept
    : polyline_(polyline)
    , step_length_(step_length)
    , dist_from_milestone_(0.0)
    , milestone_(milestone) {}

UniformIterator& UniformIterator::SetStepLength(double step_length) noexcept {
    step_length_ = step_length;
    return *this;
}

Pose UniformIterator::operator*() const noexcept { return GetPose(); }

UniformIterator& UniformIterator::operator++() noexcept {
    double step_left = step_length_;
    while (milestone_ + 1 != polyline_->end()) {
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

UniformIterator UniformIterator::operator++(int) noexcept {
    UniformIterator it = *this;
    ++it;
    Swap(it);
    return it;
}

bool operator==(const UniformIterator& first, const UniformIterator& second) noexcept {
    return first.milestone_ == second.milestone_ &&
           first.dist_from_milestone_ == second.dist_from_milestone_;
}

bool operator!=(const UniformIterator& first, const UniformIterator& second) noexcept {
    return !(first == second);
}

geom::Pose UniformIterator::GetPose() const noexcept {
    geom::AngleVec2 dir =
        (milestone_ + 1 == polyline_->end()
             ? AngleVec2::fromVector(*milestone_ - *(milestone_ - 1))
             : AngleVec2::fromVector(*(milestone_ + 1) - *milestone_));
    return geom::Pose(*milestone_ + dist_from_milestone_ * dir.vec(), dir);
}

void UniformIterator::Swap(UniformIterator& other) noexcept {
    std::swap(step_length_, other.step_length_);
    std::swap(dist_from_milestone_, other.dist_from_milestone_);
    std::swap(milestone_, other.milestone_);
}

}  // namespace truck::geom