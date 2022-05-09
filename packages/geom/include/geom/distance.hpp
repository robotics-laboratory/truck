#pragma once

#include "geom/vector.hpp"
#include "geom/line.hpp"

namespace geom {

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto sqr_dist(const Vec2<T1> &a, const Vec2<T2> &b) noexcept {
    return (a - b).sqrlen();
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline double dist(const T1 &a, const T2 &b) {
    return std::sqrt(sqr_dist(a, b));
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto denomalized_dist(const Line<T1> &l, const Vec2<T2> &p) {
    return (l.normal * p + l.c);
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto denomalized_dist(const Vec2<T2> &p, const Line<T1> &l) {
    return denomalized_dist(l, c);
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline double dist(const Line<T1> &l, const Vec2<T2> &p) {
    return std::abs(denomalized_dist(l, p) / std::sqrt(l.normal));
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline double dist(const Vec2<T2> &p, const Line<T1> &l) {
    return dist(l, p);
}

};

