#pragma once

#include "geom/vector.h"
#include "geom/pose.h"

#include <vector>
#include <iterator>

namespace truck::geom {

class UniformIterator;

struct Polyline final : public std::vector<Vec2> {
    using vector::vector;

    Polyline(std::vector<Vec2> other);

    Polyline& operator=(std::vector<Vec2> other) &;

    friend UniformIterator;

    UniformIterator ubegin() const noexcept;

    UniformIterator ubegin(double step_length) const noexcept;

    UniformIterator uend() const noexcept;
};

class UniformIterator {
  public:
    using IteratorCategory = std::forward_iterator_tag;
    using Container = Polyline;
    using ContainerIterator = Container::const_iterator;

    UniformIterator() = delete;

    UniformIterator(const Container* polyline) noexcept;

    UniformIterator(const Container* polyline, double step_length) noexcept;

    UniformIterator(const Container* polyline, ContainerIterator milestone) noexcept;

    UniformIterator(
        const Container* polyline, double step_length, ContainerIterator milestone) noexcept;

    UniformIterator& SetStepLength(double step_length) noexcept;

    Pose operator*() const noexcept;

    UniformIterator& operator++() noexcept;

    UniformIterator operator++(int) noexcept;

    friend bool operator==(const UniformIterator& first, const UniformIterator& second) noexcept;

    friend bool operator!=(const UniformIterator& first, const UniformIterator& second) noexcept;

  private:
    geom::Pose GetPose() const noexcept;

    void Swap(UniformIterator& other) noexcept;

    const Container* polyline_ = nullptr;

    double step_length_ = 1.0;

    double dist_from_milestone_ = 0.0;
    ContainerIterator milestone_;
};

}  // namespace truck::geom
