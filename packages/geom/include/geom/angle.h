#pragma once

#include <cmath>
#include <ostream>

namespace truck::geom {

class Angle {
  public:
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

    constexpr Angle& operator*=(double x) {
        value_ *= x;
        return *this;
    }

    constexpr Angle& operator/=(double x) {
        value_ /= x;
        return *this;
    }

    constexpr Angle zero() const { return Angle{0}; }

  private:
    double value_;
};

namespace literals {

constexpr Angle operator"" _rad(unsigned long long radians) { return Angle::fromRadians(radians); }
constexpr Angle operator"" _rad(long double radians) { return Angle::fromRadians(radians); }

constexpr Angle operator"" _deg(unsigned long long degrees) { return Angle::fromDegrees(degrees); }
constexpr Angle operator"" _deg(long double degrees) { return Angle::fromDegrees(degrees); }

}  // namespace literals

constexpr Angle operator/(Angle a, double v) {
    a /= v;
    return a;
}

inline constexpr bool operator<(Angle left, Angle right) {
    return left.radians() < right.radians();
}

inline constexpr bool operator<=(Angle left, Angle right) {
    return left.radians() <= right.radians();
}

inline constexpr bool operator>(Angle left, Angle right) {
    return left.radians() > right.radians();
}

inline constexpr bool operator>=(Angle left, Angle right) {
    return left.radians() >= right.radians();
}

inline std::ostream& operator<<(std::ostream& out, const Angle& angle) {
    return out << angle.degrees() << "deg";
}

constexpr Angle PI = Angle(M_PI);
constexpr Angle PI_2 = PI / 2;
constexpr Angle PI_3 = PI / 3;
constexpr Angle PI_4 = PI / 4;
constexpr Angle PI_6 = PI / 6;

}  // namespace truck::geom