#pragma once

#include "geom/common.hpp"
#include "geom/vector.hpp"

namespace geom {

// ax + by + c = 0
template <class T>
struct Line {
    T a, b, c;

    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2<T> normal() const noexcept { return {a, b}; }

    Line(T a = 1, T b = 0, T c = 0) : a(a), b(b), c(c) {}
    Line(Vec2<T> norm, T c) : a(norm.x), b(norm.y), c(c) {}

    [[gnu::always_inline, nodiscard, gnu::pure]] static Line fromTwoPoints(
        const Vec2<T> &p1, const Vec2<T> &p2) noexcept {
        Vec2<T> norm{p2.y - p1.y, p1.x - p2.x};
        return Line(norm, -dot(p1, norm));
    }

    [[gnu::always_inline, nodiscard, gnu::pure]] static Line fromPointAndNormal(
        const Vec2<T> &p1, const Vec2<T> &norm) noexcept {
        return Line(norm, -dot(p1, norm));
    }

    [[gnu::always_inline, nodiscard, gnu::pure]] static Line fromPointAndCollinear(
        const Vec2<T> &p1, const Vec2<T> &coll) noexcept {
        Vec2<T> norm{coll.y, -coll.x};
        return Line(norm, -dot(p1, norm));
    }

    [[gnu::always_inline, nodiscard, gnu::pure]] bool operator==(const Line &other) const noexcept {
        return a.normal() * b.c == b.normal() * a.c;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] bool operator!=(const Line &other) const noexcept {
        return !(*this == other);
    }
};

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline bool near(const Line<T1> &a, const Line<T2> &b,
                                                              double eps = 0) noexcept {
    return near(a.normal() * b.c, b.normal() * a.c, eps);
}

template <class T>
inline std::ostream &operator<<(std::ostream &out, const Line<T> &l) {
    return out << "Line(" << l.a << ", " << l.b << ", " << l.c << ")";
}

using Linef = Line<float>;
using Lined = Line<double>;
using Linei = Line<int>;
using Linei32 = Line<int32_t>;
using Linei64 = Line<int64_t>;

}  // namespace geom
