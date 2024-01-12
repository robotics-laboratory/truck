#pragma once

#include "geom/uniform_stepper.h"
#include "geom/vector.h"

#include <vector>

namespace truck::geom {

struct Polyline final : public std::vector<Vec2> {
    using vector::vector;

    using vector<Vec2>::operator=;

    UniformStepper<Polyline> ubegin() const noexcept;

    UniformStepper<Polyline> ubegin(double step_length) const noexcept;

    UniformStepper<Polyline> uend() const noexcept;
};

}  // namespace truck::geom
