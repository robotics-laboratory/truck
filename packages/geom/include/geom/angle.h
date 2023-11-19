#pragma once

#include <cmath>
#include <ostream>

namespace truck::geom {

class Angle {
  public:
    constexpr Angle() = default;

    constexpr explicit Angle(double rad) : value_(rad) {}

    static constexpr Angle fromRadians(double rad) noexcept { return Angle{rad}; }

    static constexpr Angle fromDegrees(double deg) noexcept { return Angle{(M_PI / 180) * deg}; }

    static constexpr Angle fromVector(double x, double y) noexcept {
        return Angle{std::atan2(y, x)};
    }

    constexpr double radians() const noexcept { return value_; }

    constexpr double degrees() const noexcept { return (180 / M_PI) * value_; }

    friend constexpr double sin(Angle angle) noexcept { return std::sin(angle.value_); }

    friend constexpr double cos(Angle angle) noexcept { return std::cos(angle.value_); }

    friend constexpr double tan(Angle angle) noexcept { return std::tan(angle.value_); }

    friend constexpr Angle abs(Angle angle) noexcept { return Angle{std::abs(angle.value_)}; }

    constexpr Angle operator-() const noexcept { return Angle{-value_}; }

    Angle& operator+=(Angle other) noexcept {
        value_ += other.value_;
        return *this;
    }

    Angle& operator-=(Angle other) noexcept {
        value_ -= other.value_;
        return *this;
    }

    Angle& operator*=(double x) noexcept {
        value_ *= x;
        return *this;
    }

    Angle& operator/=(double x) {
        value_ /= x;
        return *this;
    }

    static constexpr Angle zero() noexcept { return Angle{0}; }

    static double _0_2PI(double radians) noexcept {
        return Angle(radians)._0_2PI().radians();
    }

    constexpr Angle _0_2PI() const noexcept {
        const double result = fmod(value_, 2 * M_PI);
        return (result < 0) ? Angle{result + 2 * M_PI} : Angle{result};
    }

    constexpr Angle _mPI_PI() const noexcept {
        const double result = fmod(value_ + M_PI, 2 * M_PI);
        return (result <= 0) ? Angle{result + M_PI} : Angle{result - M_PI};
    }

  private:
    double value_{0};
};

namespace literals {

constexpr Angle operator"" _rad(unsigned long long radians) { return Angle::fromRadians(radians); }
constexpr Angle operator"" _rad(long double radians) { return Angle::fromRadians(radians); }

constexpr Angle operator"" _deg(unsigned long long degrees) { return Angle::fromDegrees(degrees); }
constexpr Angle operator"" _deg(long double degrees) { return Angle::fromDegrees(degrees); }

}  // namespace literals

constexpr Angle asin(double x) noexcept { return Angle{std::asin(x)}; }

constexpr Angle acos(double x) noexcept { return Angle{std::acos(x)}; }

constexpr Angle atan(double x) noexcept { return Angle{std::atan(x)}; }

constexpr Angle atan(double x, double y) noexcept { return Angle{std::atan2(x, y)}; }

constexpr Angle operator+(Angle a, Angle b) noexcept { return Angle{a.radians() + b.radians()}; }

constexpr Angle operator-(Angle a, Angle b) noexcept { return Angle{a.radians() - b.radians()}; }

constexpr Angle operator*(double v, Angle a) noexcept { return Angle{v * a.radians()}; }

constexpr Angle operator*(Angle a, double v) noexcept { return v * a; }

constexpr Angle operator/(Angle a, double v) noexcept { return Angle{a.radians() / v}; }

constexpr bool operator<(Angle left, Angle right) noexcept {
    return left.radians() < right.radians();
}

constexpr bool operator<=(Angle left, Angle right) noexcept {
    return left.radians() <= right.radians();
}

constexpr bool operator>(Angle left, Angle right) noexcept {
    return left.radians() > right.radians();
}

constexpr bool operator>=(Angle left, Angle right) noexcept {
    return left.radians() >= right.radians();
}

bool equal(Angle a, Angle b, double eps) noexcept;

std::ostream& operator<<(std::ostream& out, const Angle& angle) noexcept;

constexpr Angle PI = Angle(M_PI);
constexpr Angle PI0 = Angle::zero();
constexpr Angle PI2 = 2 * PI;
constexpr Angle PI_2 = PI / 2;
constexpr Angle PI_3 = PI / 3;
constexpr Angle PI_4 = PI / 4;
constexpr Angle PI_6 = PI / 6;

}  // namespace truck::geom
