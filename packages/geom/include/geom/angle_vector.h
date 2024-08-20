#pragma once

#include "geom/angle.h"
#include "geom/vector.h"

#include <ostream>

namespace truck::geom {

/*
 * Unit vector 2D
 *
 * Represents a angle (2D orientation) as a double-precision unit vector.
 * May be used for efficient rotation of vectors or pos directions.
 */

class AngleVec2 {
  public:
    constexpr AngleVec2() = default;

    AngleVec2(Angle a) : vec_(Vec2::fromAngle(a)), angle_(a) {}

    constexpr operator Angle() const noexcept { return angle(); }

    constexpr operator Vec2() const noexcept { return vec(); }

    static AngleVec2 fromRadians(double rad) noexcept {
        return {Vec2::fromAngle(Angle(rad)), Angle(rad)};
    }

    static AngleVec2 fromVectorUnsafe(Vec2 v) noexcept { return {v, Angle::fromVector(v.x, v.y)}; }

    static AngleVec2 fromVectorUnsafe(double x, double y) noexcept {
        return fromVectorUnsafe({x, y});
    }

    static AngleVec2 fromVector(Vec2 v) noexcept { return fromVectorUnsafe(v / v.len()); }

    static AngleVec2 fromVector(double x, double y) noexcept { return fromVector({x, y}); }

    AngleVec2& operator+=(const AngleVec2& other) noexcept {
        vec_ = other.apply(vec_);
        angle_ += other.angle_;

        return *this;
    }

    AngleVec2& operator-=(const AngleVec2& other) noexcept {
        vec_ = other.inv().apply(vec_);
        return *this;
    }

    constexpr AngleVec2 operator+(const AngleVec2& other) const noexcept {
        return {other.apply(vec_), angle_ + other.angle_};
    }

    constexpr AngleVec2 operator-(const AngleVec2& other) const noexcept {
        return {other.inv().apply(vec_), angle_ - other.angle_};
    }

    constexpr AngleVec2 inv() const noexcept { return {{vec_.x, -vec_.y}, -angle_}; }

    constexpr AngleVec2 operator-() const noexcept { return inv(); }

    static constexpr AngleVec2 axisX() noexcept { return {{1., 0.}, PI0}; }

    static constexpr AngleVec2 axisY() noexcept { return {{0., 1.}, PI_2}; }

    // Apply rotation to input
    constexpr Vec2 apply(Vec2 v) const noexcept {
        return {vec_.x * v.x - vec_.y * v.y, vec_.y * v.x + vec_.x * v.y};
    }

    constexpr AngleVec2 apply(AngleVec2 v) const noexcept {
        return {apply(v.vec()), v.angle() + angle()};
    }

    constexpr AngleVec2 left() const noexcept { return {vec_.left(), angle_ + PI_2}; }

    constexpr AngleVec2 right() const noexcept { return {vec_.right(), angle_ - PI_2}; }

    constexpr const Angle& angle() const noexcept { return angle_; }

    constexpr double x() const noexcept { return vec_.x; }
    constexpr double y() const noexcept { return vec_.y; }

    constexpr const Vec2& vec() const noexcept { return vec_; }

  private:
    constexpr AngleVec2(Vec2 v, Angle a) : vec_(v), angle_(a) {}

    Vec2 vec_{1, 0};
    Angle angle_{0};
};

bool equal(const AngleVec2& a, const AngleVec2& b, double eps = 0) noexcept;

constexpr Vec2 operator*(const AngleVec2& a, double c) noexcept { return a.vec() * c; }

constexpr Vec2 operator*(double c, const AngleVec2& a) noexcept { return a * c; }

constexpr Vec2 operator/(const AngleVec2& a, double c) noexcept { return a.vec() / c; }

std::ostream& operator<<(std::ostream& out, const AngleVec2& a) noexcept;

AngleVec2 interpolate(const AngleVec2& a, const AngleVec2& b, double t) noexcept;

}  // namespace truck::geom
