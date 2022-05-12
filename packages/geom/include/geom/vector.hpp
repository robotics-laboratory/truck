#pragma once

#include <cinttypes>
#include <cmath>
#include <iostream>
#include <type_traits>

#include "geom/common.hpp"

namespace geom {

template <class T>
struct Vec2 {
    T x, y;
    template <class U, std::enable_if_t<!std::is_same_v<U, std::common_type_t<T, U>>, bool> = true>
    [[gnu::always_inline]] explicit operator Vec2<U>() const noexcept {
        return Vec2<U>{static_cast<U>(x), static_cast<U>(y)};
    }
    template <class U, std::enable_if_t<std::is_same_v<U, std::common_type_t<T, U>>, bool> = true>
    [[gnu::always_inline]] operator Vec2<U>() const noexcept {
        return Vec2<U>{static_cast<U>(x), static_cast<U>(y)};
    }
    [[gnu::always_inline]] Vec2(T x = 0, T y = 0): x(x), y(y) {}
    template<class V>
    [[gnu::always_inline]] explicit Vec2(const V& v) noexcept: x(v.x), y(v.y) {}
    [[gnu::always_inline]] Vec2 &operator+=(const Vec2 &other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    [[gnu::always_inline]] Vec2 &operator-=(const Vec2 &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    template <class U>
    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2<std::common_type_t<T, U>> operator+(
        const Vec2<U> &other) const noexcept {
        return {x + other.x, y + other.y};
    }
    template <class U>
    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2<std::common_type_t<T, U>> operator-(
        const Vec2<U> &other) const noexcept {
        return {x - other.x, y - other.y};
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] T lenSq() const noexcept { return x * x + y * y; }
    [[gnu::always_inline, nodiscard, gnu::pure]] auto len() const noexcept {
        return std::sqrt(lenSq());
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2 operator-() const noexcept {
        return Vec2{-x, -y};
    }
    [[gnu::always_inline]] Vec2 &operator*=(T c) {
        x *= c;
        y *= c;
        return *this;
    }
    template<class Dummy = T, std::enable_if_t<std::is_floating_point_v<Dummy>, bool> = true>
    [[gnu::always_inline]] Vec2 &operator/=(T c) {
        x /= c;
        y /= c;
        return *this;
    }
    template <class U>
    [[gnu::always_inline, nodiscard, gnu::pure]] auto rotate(U radians) const {
        using Ret = std::conditional_t<std::is_floating_point_v<T>, T, double>;
        using Ang = std::common_type_t<Ret, U>;
        auto sn = std::sin(static_cast<Ang>(radians));
        auto cs = std::cos(static_cast<Ang>(radians));
        return Vec2<Ret>{x * cs - y * sn, x * sn + y * cs};
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2 left() const noexcept {
        Vec2 ans = *this;
        std::swap(ans.x, ans.y);
        ans.x = -ans.x;
        return ans;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2 right() const noexcept {
        Vec2 ans = *this;
        std::swap(ans.x, ans.y);
        ans.y = -ans.y;
        return ans;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] auto radians() const { return std::atan2(y, x); }
    [[gnu::always_inline, nodiscard, gnu::pure]] bool operator==(const Vec2 &other) const noexcept {
        return x == other.x && y == other.y;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] bool operator!=(const Vec2 &other) const noexcept {
        return !(*this == other);
    }
};

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline Vec2<std::common_type_t<T1, T2>> operator*(
    const Vec2<T1> &v, T2 c) noexcept {
    return {v.x * c, v.y * c};
}

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto operator*(T2 c,
                                                                   const Vec2<T1> &v) noexcept {
    return v * c;
}

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto operator/(const Vec2<T1> &v,
                                                                   T2 c) noexcept {
    using Ret = std::conditional_t<std::is_integral_v<T1> && std::is_integral_v<T2>, double,
                                   std::common_type_t<T1, T2>>;
    return Vec2<Ret>{static_cast<Ret>(v.x) / c, static_cast<Ret>(v.y) / c};
}

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline bool near(const Vec2<T1> &a, const Vec2<T2> &b,
                                                              double eps = 0) noexcept {
    return near(a.x, b.x, eps) && near(a.y, b.y, eps);
}

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto dot(const Vec2<T1> &a,
                                                             const Vec2<T2> &b) noexcept {
    return a.x * b.x + a.y * b.y;
}

template <class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto cross(const Vec2<T1> &a,
                                                               const Vec2<T2> &b) noexcept {
    return a.x * b.y - a.y * b.x;
}

template <class T>
inline std::ostream &operator<<(std::ostream &out, const Vec2<T> &v) {
    return out << "(" << v.x << ", " << v.y << ")";
}

using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;
using Vec2i = Vec2<int>;
using Vec2i32 = Vec2<int32_t>;
using Vec2i64 = Vec2<int64_t>;

}  // namespace geom
