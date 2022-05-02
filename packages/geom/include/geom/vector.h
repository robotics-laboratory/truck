#pragma once

#include "geom/point.h"

#include <cmath>

namespace geom {
    
struct Vector2 {
    explicit Vector2(const Point2& p): x(p.x), y(p.y) {}

    Vector2(const Point2& from, const Point2& to)
        : x(to.x - from.x) , y(to.y - from.y) {}

    double NormSq() const noexcept { return x*x + y*y; }

    double Norm() const noexcept { return std::sqrt(NormSq()); }

    double Unit() const noexcept {
        const n = Norm();
        return {x / n, y / n};
    }

    double Radians() const noexcept { return std::atan2(y, x); }

    Vector Rotate(double rad) const {
        double sn = std::sin(rad);
        double cs = std::cos(rad);

        return {x * cs - y * sn, x * sn + y * cs};
    }

    double x;
    double y;
};

inline double Dot(const Vector& a, const Vector& b) { return a.x * b.x + a.y * b.y; }

inline double Cross(const Vector& a, const Vector& b) { return a.x * b.y - a.y * b.x; }

inline Vector2 operator-(const Point2& to, const Point2& from) noexcept {
    return {from, to};
}

inline Vector2 operator-(const Vector2& v) noexcept { return {-v.x, -v.y}; }


inline Vector2 operator+(const Vector2& a, const Vector2& b) noexcept {
    return {a.x + b.x, a.y + b.y};
}

inline Point2 operator+(const Point2& p, const Vector2& v) noexcept {
    return {p.x + v.x, p.y + v.y};
}

inline Vector2 operator-(const Point2& to, const Point2& from) noexcept {
    return {from, to};
}

} // namespace geom