#pragma once

#include "geom/vector.hpp"
#include "geom/common.hpp"

namespace geom {

// ax + by + c = 0
template<class T>
struct Line {
    union {
        struct {
            T a, b;
        };
        Vec2<T> normal;
    };
    T c;

    Line(T a = 1, T b = 0, T c = 0): a(a), b(b), c(c) {}
    Line(Vec2<T> norm, T c): norm(norm), c(c) {}

    static [[gnu::always_inline, nodiscard, gnu::pure]] Line from_two_points(const Vec2<T> &p1, const Vec2<T> &p2) noexcept {
        Vec2 norm{p2.y - p1.y, p1.x - p2.x};
        return Line(norm, -(p1 * normal));
    }

    static [[gnu::always_inline, nodiscard, gnu::pure]] Line from_point_and_normal(const Vec2<T> &p1, const Vec2<T> &norm) noexcept {
        return Line(norm, -(p1 * normal));
    }

    static [[gnu::always_inline, nodiscard, gnu::pure]] Line from_point_and_collinear(const Vec2<T> &p1, const Vec2<T> &coll) noexcept {
        Vec2<T> norm{coll.y, -coll.x};
        return Line(norm, -(p1 * normal));
    }
};

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline bool eq(const Line<T1>& a, const Line<T2>& b, double eps = 0) noexcept {
    return eq(a.norm * b.c, b.norm * a.c, eps);
}

using Linef = Line<float>;
using Lined = Line<double>;
using Linei = Line<int>;
using Linei32 = Line<int32_t>;
using Linei64 = Line<int64_t>;

};
