#pragma once

#include <cmath>
#include <ostream>

namespace truck::geom {

class Angle {
  public:
    constexpr Angle() : value_(0) {}

    explicit constexpr Angle(double rad) : value_(rad) {}

    explicit operator double() { return value_; }

    static constexpr Angle fromRadians(double rad) { return Angle{rad}; }

    static constexpr Angle fromDegrees(double deg) { return Angle{(M_PI / 180) * deg}; }

    static constexpr Angle fromVector(double x, double y) { return Angle{std::atan2(x, y)}; }

    constexpr double radians() const { return value_; }

    constexpr double degrees() const { return (180 / M_PI) * value_; }

    friend constexpr double sin(Angle angle) { return std::sin(angle.value_); }

    friend constexpr double cos(Angle angle) { return std::cos(angle.value_); }

    friend constexpr double tan(Angle angle) { return std::tan(angle.value_); }

    friend constexpr Angle abs(Angle angle) { return Angle{std::abs(angle.value_)}; }

    constexpr Angle operator-() const { return Angle{-value_}; }

    constexpr Angle& operator*=(double x) {
        value_ *= x;
        return *this;
    }

    constexpr Angle& operator/=(double x) {
        value_ /= x;
        return *this;
    }

    constexpr Angle zero() const { return Angle{0}; }

    constexpr Angle _0_2PI() const {
        const double result = fmod(value_, 2 * M_PI);
        return (result < 0) ? Angle{result + 2 * M_PI} : Angle{result};
    }

    constexpr Angle _mPI_PI() const {
        const double result = fmod(value_ + M_PI, 2 * M_PI);
        return (result <= 0) ? Angle{result + M_PI} : Angle{result - M_PI};
    }

  private:
    double value_;
};

namespace literals {

inline constexpr Angle operator"" _rad(unsigned long long radians) {
    return Angle::fromRadians(radians);
}

inline constexpr Angle operator"" _rad(long double radians) { return Angle::fromRadians(radians); }

inline constexpr Angle operator"" _deg(unsigned long long degrees) {
    return Angle::fromDegrees(degrees);
}

inline constexpr Angle operator"" _deg(long double degrees) { return Angle::fromDegrees(degrees); }

}  // namespace literals

inline constexpr  Angle asin(double x) noexcept { return Angle{std::asin(x)}; }

inline constexpr Angle acos(double x) noexcept { return Angle{std::acos(x)}; }

inline constexpr Angle atan(double x) noexcept { return Angle{std::atan(x)}; }

inline constexpr Angle atan(double x, double y) noexcept { return Angle{std::atan2(x, y)}; }

inline constexpr Angle operator+(Angle a, Angle b) noexcept {
    return Angle{a.radians() + b.radians()};
}

inline constexpr Angle operator-(Angle a, Angle b) noexcept {
    return Angle{a.radians() - b.radians()};
}

inline constexpr Angle operator*(double v, Angle a) noexcept { return Angle{v * a.radians()}; }

inline constexpr Angle operator*(Angle a, double v) noexcept { return v * a; }

inline constexpr Angle operator/(Angle a, double v) noexcept { return Angle{a.radians() / v}; }

inline constexpr bool operator<(Angle left, Angle right) noexcept {
    return left.radians() < right.radians();
}

inline constexpr bool operator<=(Angle left, Angle right) noexcept {
    return left.radians() <= right.radians();
}

inline constexpr bool operator>(Angle left, Angle right) noexcept {
    return left.radians() > right.radians();
}

inline constexpr bool operator>=(Angle left, Angle right) noexcept {
    return left.radians() >= right.radians();
}

std::ostream& operator<<(std::ostream& out, const Angle& angle) noexcept;

bool equal(Angle a, Angle b, double eps) noexcept;

constexpr Angle PI = Angle(M_PI);
constexpr Angle PI2 = 2 * PI;
constexpr Angle PI_2 = PI / 2;
constexpr Angle PI_3 = PI / 3;
constexpr Angle PI_4 = PI / 4;
constexpr Angle PI_6 = PI / 6;

}  // namespace truck::geom