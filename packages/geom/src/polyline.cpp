#include "geom/polyline.h"

namespace truck::geom {

UniformIterator Polyline::ubegin() const noexcept { return UniformIterator(this); }

UniformIterator Polyline::ubegin(double step_length) const noexcept {
    return UniformIterator(this, step_length);
}

UniformIterator Polyline::uend() const noexcept { return UniformIterator(this, this->end() - 1); }

UniformIterator Polyline::uend(double step_length) const noexcept {
    return UniformIterator(this, step_length, this->end() - 1);
}

UniformIterator::UniformIterator(const Container* polyline) noexcept
    : polyline_(polyline)
    , step_length_(1.0)
    , current_iterator_(polyline_->begin())
    , current_point_(*current_iterator_) {}

UniformIterator::UniformIterator(const Container* polyline, double step_length) noexcept
    : polyline_(polyline)
    , step_length_(step_length)
    , current_iterator_(polyline_->begin())
    , current_point_(*current_iterator_) {}

UniformIterator::UniformIterator(
    const Container* polyline, ContainerIterator current_iterator) noexcept
    : polyline_(polyline)
    , step_length_(1.0)
    , current_iterator_(current_iterator)
    , current_point_(*current_iterator_) {}

UniformIterator::UniformIterator(
    const Container* polyline, double step_length, ContainerIterator current_iterator) noexcept
    : polyline_(polyline)
    , step_length_(step_length)
    , current_iterator_(current_iterator)
    , current_point_(*current_iterator_) {}

UniformIterator& UniformIterator::SetStepLength(double step_length) noexcept {
    step_length_ = step_length;
    return *this;
}

const Vec2& UniformIterator::operator*() const noexcept { return current_point_; }

const Vec2* UniformIterator::operator->() const noexcept { return &current_point_; }

UniformIterator& UniformIterator::operator++() noexcept {
    double step_left = step_length_;
    while (current_iterator_ + 1 != polyline_->end()) {
        Vec2 dir = (*(current_iterator_ + 1) - current_point_);
        if (dir.len() > step_left + precision) {
            current_point_ += dir.unit() * step_left;
            break;
        }
        step_left -= dir.len();
        ++current_iterator_;
        current_point_ = *current_iterator_;
    }
    return *this;
}

UniformIterator UniformIterator::operator++(int) noexcept {
    UniformIterator it = *this;
    ++it;
    Swap(it);
    return it;
}

UniformIterator& UniformIterator::operator--() noexcept {
    double step_left = step_length_;
    while (current_iterator_ != polyline_->begin()) {
        Vec2 dir = (*(current_iterator_ - 1) - current_point_);
        if (dir.len() > step_left + precision) {
            current_point_ += dir.unit() * step_left;
            break;
        }
        step_left -= dir.len();
        --current_iterator_;
        current_point_ = *current_iterator_;
    }
    return *this;
}

UniformIterator UniformIterator::operator--(int) noexcept {
    UniformIterator it = *this;
    --it;
    Swap(it);
    return it;
}

bool operator==(const UniformIterator& first, const UniformIterator& second) noexcept {
    return first.current_iterator_ == second.current_iterator_ &&
           abs(first.current_point_.x - second.current_point_.x) < UniformIterator::precision &&
           abs(first.current_point_.y - second.current_point_.y) < UniformIterator::precision;
}

bool operator!=(const UniformIterator& first, const UniformIterator& second) noexcept {
    return !(first == second);
}

void UniformIterator::Swap(UniformIterator& other) noexcept {
    std::swap(current_iterator_, other.current_iterator_);
    std::swap(current_point_, other.current_point_);
}

}  // namespace truck::geom