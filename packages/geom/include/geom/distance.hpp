#pragma once

#include "geom/vector.hpp"
#include "geom/line.hpp"

namespace geom {

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto sqr_dist(const Vec2<T1> &a, const Vec2<T2> &b) noexcept {
    return (a - b).sqr_len();
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline double dist(const T1 &a, const T2 &b) {
    return std::sqrt(sqr_dist(a, b));
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto denormalized_dist(const Line<T1> &l, const Vec2<T2> &p) {
    return (dot(l.normal(), p) + l.c);
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto denormalized_dist(const Vec2<T2> &p, const Line<T1> &l) {
    return denormalized_dist(l, p);
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline double dist(const Line<T1> &l, const Vec2<T2> &p) {
    return std::abs(denormalized_dist(l, p) / l.normal().len());
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline double dist(const Vec2<T2> &p, const Line<T1> &l) {
    return dist(l, p);
}

}

