#pragma once

#include "geom/vector.h"

#include <vector>
#include <iterator>

namespace truck::geom {

class UniformIterator;

struct Polyline final : public std::vector<Vec2> {
    using vector::vector;

    friend UniformIterator;

    UniformIterator ubegin() const noexcept;

    UniformIterator ubegin(const double step_length) const noexcept;

    UniformIterator uend() const noexcept;

    UniformIterator uend(const double step_length) const noexcept;
};

class UniformIterator {
  public:
    static constexpr double precision = 1e-7;

    using IteratorCategory = std::bidirectional_iterator_tag;
    using Container = Polyline;
    using ContainerIterator = Container::const_iterator;

    UniformIterator() = delete;

    UniformIterator(const Container* polyline) noexcept;

    UniformIterator(const Container* polyline, const double step_length) noexcept;

    UniformIterator(const Container* polyline, ContainerIterator current_iterator) noexcept;

    UniformIterator(
        const Container* polyline, const double step_length,
        ContainerIterator current_iterator) noexcept;

    UniformIterator& SetStepLength(const double step_length) noexcept;

    const Vec2& operator*() const noexcept;

    const Vec2* operator->() const noexcept;

    UniformIterator& operator++() noexcept;

    UniformIterator operator++(int) noexcept;

    UniformIterator& operator--() noexcept;

    UniformIterator operator--(int) noexcept;

    friend bool operator==(const UniformIterator& first, const UniformIterator& second) noexcept;

    friend bool operator!=(const UniformIterator& first, const UniformIterator& second) noexcept;

  private:
    void Swap(UniformIterator& other) noexcept;

    const Container* polyline_ = nullptr;

    double step_length_ = 1.0;
    ContainerIterator current_iterator_;
    Vec2 current_point_;
};

}  // namespace truck::geom
