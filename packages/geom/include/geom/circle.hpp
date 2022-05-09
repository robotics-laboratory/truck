#pragma once

#include "geom/vector.hpp"
#include "geom/common.hpp"

namespace geom {

template<class T>
struct Circle {
    Vec2<T> center;
    T radius;
};

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline bool eq(const Circle<T1>& a, const Circle<T2>& b, double eps = 0) noexcept {
    return eq(a.radius, b.radius, eps) && eq(a.center, b.center, eps);
}

};
