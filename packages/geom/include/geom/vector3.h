#pragma once

#include <cmath>
#include <cmath>
#include <ostream>

namespace truck::geom {

struct Vec3 {
    constexpr Vec3() : x(0), y(0), z(0) {}

    constexpr Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    constexpr Vec3(Vec2 vec2, double z) : x(vec2.x), y(vec2.y), z(z) {}

    constexpr Vec3(Vec2 vec2) : Vec3(vec2, 0) {}

    Vec3& operator+=(const Vec3& other) noexcept {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) noexcept {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    constexpr Vec3 operator+(const Vec3& other) const noexcept {
        return {x + other.x, y + other.y, z + other.z};
    }

    constexpr Vec3 operator-(const Vec3& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }

    double lenSq() const noexcept { return x * x + y * y + z * z; }

    double len() const noexcept { return std::hypot(x, y, z); }

    Vec3 unit() const noexcept {
        const double l = len();
        return Vec3{x / l, y / l, z / l};
    }

    static constexpr Vec3 axisX() noexcept { return Vec3{1, 0, 0}; }

    static constexpr Vec3 axisY() noexcept { return Vec3{0, 1, 0}; }

    static constexpr Vec3 axisZ() noexcept { return Vec3{0, 0, 1}; }

    constexpr Vec3 operator-() const noexcept { return inv(); }

    Vec3& operator*=(double c) noexcept {
        x *= c;
        y *= c;
        z *= c;
        return *this;
    }

    Vec3& operator/=(double c) noexcept {
        x /= c;
        y /= c;
        z /= c;
        return *this;
    }

    constexpr Vec3 inv() const noexcept { return {-x, -y, -z}; }

    double x, y, z;
};

constexpr Vec3 operator*(Vec3 v, double c) noexcept { return {v.x * c, v.y * c, v.z * c}; }

constexpr Vec3 operator*(double c, Vec3 v) noexcept { return v * c; }

constexpr Vec3 operator/(const Vec3& v, double c) noexcept { return {v.x / c, v.y / c, v.z / c}; }

constexpr double dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline double lenSq(const Vec3& v) noexcept { return v.lenSq(); }

inline double len(const Vec3& v) noexcept { return v.len(); }

bool equal(const Vec3& a, const Vec3& b, double eps = 0) noexcept;

Vec3 interpolate(const Vec3& a, const Vec3& b, double t) noexcept;

std::ostream& operator<<(std::ostream& out, const Vec3& v);

}  // namespace truck::geom
