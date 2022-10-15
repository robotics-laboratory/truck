#pragma once

#include "geom/angle.h"

#include <cmath>
#include <ostream>

namespace truck::geom {

struct Vec2 {
    Vec2() = default;

    Vec2(double x, double y) : x(x), y(y) {}

    explicit Vec2(Angle angle) : x(cos(angle)), y(sin(angle)) {}

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

    double lenSq() const noexcept { return x * x + y * y; }

    double len() const noexcept { return std::sqrt(lenSq()); }

    Vec2 operator-() const noexcept { return Vec2{-x, -y}; }

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

    Vec2 left() const noexcept { return {-y, x}; }

    Vec2 right() const noexcept { return {y, -x}; }

    Angle angle() const { return atan(y, x); }

    double x, y;
};

Vec2 operator*(const Vec2& v, double c) noexcept;

Vec2 operator*(double c, const Vec2& v) noexcept;

Vec2 operator/(const Vec2& v, double c) noexcept;

bool equal(const Vec2& a, const Vec2& b, double eps = 0) noexcept;

double dot(const Vec2& a, const Vec2& b) noexcept;

double cross(const Vec2& a, const Vec2& b) noexcept;

std::ostream& operator<<(std::ostream& out, const Vec2& v);

}  // namespace truck::geom
