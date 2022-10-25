#pragma once

#include "geom/angle.h"

#include <cmath>
#include <ostream>

namespace truck::geom {

struct Vec2 {
    constexpr Vec2(): x(0), y(0) {}

    constexpr Vec2(double x, double y) : x(x), y(y) {}

    constexpr explicit Vec2(Angle angle) : x(cos(angle)), y(sin(angle)) {}

    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2 operator+(const Vec2& other) const noexcept { return {x + other.x, y + other.y}; }

    Vec2 operator-(const Vec2& other) const noexcept { return {x - other.x, y - other.y}; }

    constexpr double lenSq() const noexcept { return x * x + y * y; }

    constexpr double len() const noexcept { return std::sqrt(lenSq()); }

    constexpr Vec2 unit() const noexcept {
        const double length = len();
        return Vec2{x / length, y / length};
    }

    constexpr Vec2 operator-() const noexcept { return Vec2{-x, -y}; }

    Vec2& operator*=(double c) {
        x *= c;
        y *= c;
        return *this;
    }

    Vec2& operator/=(double c) {
        x /= c;
        y /= c;
        return *this;
    }

    Vec2 rotate(Angle angle) const {
        auto sn = sin(angle);
        auto cs = cos(angle);

        return {x * cs - y * sn, x * sn + y * cs};
    }

    constexpr Vec2 left() const noexcept { return {-y, x}; }

    constexpr Vec2 right() const noexcept { return {y, -x}; }

    constexpr Angle angle() const { return atan(y, x); }

    double x, y;
};

Vec2 operator*(const Vec2& v, double c) noexcept;

Vec2 operator*(double c, const Vec2& v) noexcept;

Vec2 operator/(const Vec2& v, double c) noexcept;

bool equal(const Vec2& a, const Vec2& b, double eps = 0) noexcept;

double dot(const Vec2& a, const Vec2& b) noexcept;

double cross(const Vec2& a, const Vec2& b) noexcept;

Angle angleBetween(const Vec2& from, const Vec2& to);

std::ostream& operator<<(std::ostream& out, const Vec2& v);

}  // namespace truck::geom
