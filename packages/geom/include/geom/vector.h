#pragma once

#include "geom/angle.h"

#include <cmath>
#include <cmath>
#include <ostream>

namespace truck::geom {

/* 2D vector
 *
 * Represents a point or vector using double-precision x and y coordinates.
 * For storing angles, it is recommended to use the Angle class.
 * However, if you need to represent directions efficiently,
 * consider using the AngleVec2 class, which is optimized for that purpose.
 */

struct Vec2 {
    constexpr Vec2() : x(0), y(0) {}

    constexpr Vec2(double x, double y) : x(x), y(y) {}

    static Vec2 fromAngle(Angle a) noexcept { return {cos(a), sin(a)}; }

    Vec2& operator=(const Vec2& other) noexcept {
        if (this == &other) {
            return *this;
        }

        x = other.x;
        y = other.y;
        return *this;
    }

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

    double lenSq() const noexcept { return x * x + y * y; }

    double len() const noexcept { return std::hypot(x, y); }

    Angle angle() const noexcept { return Angle::fromVector(x, y); }

    Vec2 unit() const noexcept {
        const double l = len();
        return Vec2{x / l, y / l};
    }

    static constexpr Vec2 axisX() noexcept { return Vec2{1, 0}; }

    static constexpr Vec2 axisY() noexcept { return Vec2{0, 1}; }

    constexpr Vec2 operator-() const noexcept { return inv(); }

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

    constexpr Vec2 inv() const noexcept { return {-x, -y}; }

    constexpr Vec2 left() const noexcept { return {-y, x}; }

    constexpr Vec2 right() const noexcept { return {y, -x}; }

    double x, y;
};

constexpr Vec2 operator*(Vec2 v, double c) noexcept { return {v.x * c, v.y * c}; }

constexpr Vec2 operator*(double c, Vec2 v) noexcept { return v * c; }

constexpr Vec2 operator/(const Vec2& v, double c) noexcept { return {v.x / c, v.y / c}; }

constexpr double dot(const Vec2& a, const Vec2& b) noexcept { return a.x * b.x + a.y * b.y; }

constexpr double cross(const Vec2& a, const Vec2& b) noexcept { return a.x * b.y - a.y * b.x; }

inline double lenSq(const Vec2& v) noexcept { return v.lenSq(); }

inline double len(const Vec2& v) noexcept { return v.len(); }

bool equal(const Vec2& a, const Vec2& b, double eps = 0) noexcept;

Angle angleBetween(const Vec2& a, const Vec2& b) noexcept;

Vec2 interpolate(const Vec2& a, const Vec2& b, double t) noexcept;

std::ostream& operator<<(std::ostream& out, const Vec2& v);

}  // namespace truck::geom
