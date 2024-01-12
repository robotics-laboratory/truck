#include "geom/polyline.h"

namespace truck::geom {

UniformStepper<Polyline> Polyline::ubegin() const noexcept { return UniformStepper(this); }

UniformStepper<Polyline> Polyline::ubegin(double step_length) const noexcept {
    return UniformStepper(this, step_length);
}

UniformStepper<Polyline> Polyline::uend() const noexcept {
    return UniformStepper(this, this->end() - 1);
}

}  // namespace truck::geom