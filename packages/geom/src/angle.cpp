#include "geom/angle.h"

#include <iomanip>

namespace truck::geom {

std::ostream& operator<<(std::ostream& out, const Angle& angle) noexcept {
    return out << std::fixed << std::setprecision(2) << angle.degrees() << "deg";
}

bool equal(Angle a, Angle b, double eps) noexcept { return abs((a - b)._mPI_PI()).radians() < eps; }

}  // namespace truck::geom