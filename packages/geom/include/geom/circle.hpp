#pragma once

#include "geom/common.hpp"
#include "geom/vector.hpp"

namespace geom {

template <class T>
struct Circle {
    Vec2<T> center;
    T radius;

    [[gnu::always_inline, nodiscard, gnu::pure]] bool operator==(const Line& other) const noexcept {
        return center == other.center && radius == other.radius;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] bool operator!=(const Line& other) const noexcept {
        return !(*this == other);
    }
};

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline bool near(const Circle<T1>& a,
                                                              const Circle<T2>& b,
                                                              double eps = 0) noexcept {
    return near(a.radius, b.radius, eps) && near(a.center, b.center, eps);
}

template <class T>
inline std::ostream& operator<<(std::ostream& out, const Circle<T>& c) {
    return out << "Circle{" << c.center << ", " << c.radius << "}";
}

using Circlef = Circle<float>;
using Circled = Circle<double>;
using Circlei = Circle<int>;
using Circlei32 = Circle<int32_t>;
using Circlei64 = Circle<int64_t>;

};  // namespace geom
