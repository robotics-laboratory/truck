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

    double len() const noexcept;
};

Polyline toSpline(const Polyline& polyline, double step, size_t degree) noexcept;

Polyline toLinearSpline(const Polyline& polyline, double step) noexcept;

Polyline toQuadraticSpline(const Polyline& polyline, double step) noexcept;

}  // namespace truck::geom
