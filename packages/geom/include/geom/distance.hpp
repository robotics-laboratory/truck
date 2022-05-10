#pragma once

#include "geom/line.hpp"
#include "geom/vector.hpp"

namespace geom {

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto distSq(const Vec2<T1> &a,
                                                                  const Vec2<T2> &b) noexcept {
    return (a - b).lenSq();
}

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline double dist(const T1 &a, const T2 &b) {
    return std::sqrt(distSq(a, b));
}

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto denormalizedDist(const Line<T1> &l,
                                                                           const Vec2<T2> &p) {
    return (dot(l.normal(), p) + l.c);
}

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto denormalizedDist(const Vec2<T2> &p,
                                                                           const Line<T1> &l) {
    return denormalizedDist(l, p);
}

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline double dist(const Line<T1> &l,
                                                                const Vec2<T2> &p) {
    return std::abs(denormalizedDist(l, p) / l.normal().len());
}

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline double dist(const Vec2<T2> &p,
                                                                const Line<T1> &l) {
    return dist(l, p);
}

}  // namespace geom
