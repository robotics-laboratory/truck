#pragma once

#include "geom/angle.h"

#include <cmath>
#include <ostream>

namespace truck::geom {

struct Vec2 {
    constexpr Vec2() : x(0), y(0) {}

    constexpr Vec2(double x, double y) : x(x), y(y) {}

    constexpr explicit Vec2(Angle angle) : x(cos(angle)), y(sin(angle)) {}

    Vec2& operator+=(const Vec2& other) noexcept {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2& operator-=(const Vec2& other) noexcept {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    constexpr Vec2 operator+(const Vec2& other) const noexcept {
        return {x + other.x, y + other.y};
    }

    constexpr Vec2 operator-(const Vec2& other) const noexcept {
        return {x - other.x, y - other.y};
    }

    constexpr double lenSq() const noexcept { return x * x + y * y; }

    constexpr double len() const noexcept { return std::sqrt(lenSq()); }

    constexpr Vec2 unit() const noexcept {
        const double length = len();
        return Vec2{x / length, y / length};
    }

    static constexpr Vec2 axisX() noexcept { return Vec2{1, 0}; }

    static constexpr Vec2 axisY() noexcept { return Vec2{0, 1}; }

    constexpr Vec2 operator-() const noexcept { return Vec2{-x, -y}; }

    Vec2& operator*=(double c) noexcept {
        x *= c;
        y *= c;
        return *this;
    }

    Vec2& operator/=(double c) noexcept {
        x /= c;
        y /= c;
        return *this;
    }

    Vec2 inv() const noexcept { return {x, -y}; }

    Vec2 rotate(const Vec2& a) const noexcept { return {x * a.x - y * a.y, x * a.y + y * a.x}; }

    Vec2 rotate(Angle angle) const noexcept { return rotate(Vec2(angle)); }

    constexpr Vec2 left() const noexcept { return {-y, x}; }

    constexpr Vec2 right() const noexcept { return {y, -x}; }

    constexpr Angle angle() const noexcept { return atan(y, x); }

    double x, y;
};

constexpr Vec2 operator*(const Vec2& v, double c) noexcept { return {v.x * c, v.y * c}; }

constexpr Vec2 operator*(double c, const Vec2& v) noexcept { return v * c; }

constexpr Vec2 operator/(const Vec2& v, double c) noexcept { return Vec2{v.x / c, v.y / c}; }

bool equal(const Vec2& a, const Vec2& b, double eps = 0) noexcept;

double dot(const Vec2& a, const Vec2& b) noexcept;

double cross(const Vec2& a, const Vec2& b) noexcept;

Angle angleBetween(const Vec2& from, const Vec2& to) noexcept;

inline double lenSq(const Vec2& v) noexcept { return v.lenSq(); }

inline double len(const Vec2& v) noexcept { return v.len(); }

Vec2 interpolate(const Vec2& a, const Vec2& b, double t) noexcept;

std::ostream& operator<<(std::ostream& out, const Vec2& v);

}  // namespace truck::geom
