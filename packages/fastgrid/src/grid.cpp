#include "fastgrid/grid.h"

namespace truck::fastgrid {

bool operator==(const Size& a, const Size& b) noexcept {
    return a.width == b.width && a.height == b.height;
}

bool operator!=(const Size& a, const Size& b) noexcept { return !(a == b); }

std::ostream& operator<<(std::ostream& out, const Size& index) noexcept {
    return out << index.width << "x" << index.height;
}

bool operator==(const Index& a, const Index& b) noexcept { return a.x == b.x && a.y == b.y; }

bool operator!=(const Index& a, const Index& b) noexcept { return !(a == b); }

std::ostream& operator<<(std::ostream& out, const Index& index) noexcept {
    return out << index.x << ":" << index.y;
}

}  // namespace truck::fastgrid
