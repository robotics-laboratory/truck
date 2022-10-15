#include "geom/angle.h"

namespace truck::geom {

std::ostream& operator<<(std::ostream& out, const Angle& angle) noexcept {
    return out << angle.degrees() << "deg";
}

bool equal(Angle a, Angle b, double eps) noexcept {
    return abs((a - b)._mPI_PI()).radians() < eps;
}

}  // namespace truck::geom