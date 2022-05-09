#pragma once

#include <cmath>
#include <type_traits>
#include <cinttypes>

#include "geom/common.hpp"

namespace geom {
    
template<class T>
struct Vec2 {
    T x, y;
    template<class U>
    [[gnu::always_inline]] operator Vec2<U>() const noexcept {
        return Vec2<U>(x, y);
    }
    [[gnu::always_inline]] Vec2 &operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }
    [[gnu::always_inline]] Vec2 &operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    template<class U>
    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2<std::common_type_t<T, U>> operator+(const Vec2<U>& other) const noexcept {
        return {x + other.x, y + other.y};
    }
    template<class U>
    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2<std::common_type_t<T, U>> operator-(const Vec2<U>& other) const noexcept {
        return {x - other.x, y - other.y};
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] T sqr_len() const noexcept {
        return x * x + y * y;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] T len() const noexcept {
        return std::sqrt(sqr_len());
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2 operator-() const noexcept {
        return Vec2(-x, -y);
    }
    [[gnu::always_inline]] Vec2 &operator*=(T c) {
        x *= c;
        y *= c;
        return *this;
    }
    [[gnu::always_inline]] Vec2 &operator/=(T c) {
        x /= c;
        y /= c;
        return *this;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2<std::common_type_t<T, U>> operator*(T c) const noexcept {
        return {x * c, y * c};
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] Vec2<std::common_type_t<T, U>> operator*(T c) const noexcept {
        return {x / c, y / c};
    }
};

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline bool eq(const Vec2<T1>& a, const Vec2<T2>& b, double eps = 0) noexcept {
    return eq(a.x, b.x, eps) && eq(a.y, b.y, eps);
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto dot(const Vec2<T1> &a, const Vec2<T2> &b) noexcept {
    return a.x * b.x + a.y * b.y;
}

template<class T1, class T2>
[[gnu::always_inline, nodiscard, gnu::pure]] inline auto cross(const Vec2<T1> &a, const Vec2<T2> &b) noexcept {
    return a.x * b.y - a.y * b.x;
}

using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;
using Vec2i = Vec2<int>;
using Vec2i32 = Vec2<int32_t>;
using Vec2i64 = Vec2<int64_t>;

} // namespace geom
