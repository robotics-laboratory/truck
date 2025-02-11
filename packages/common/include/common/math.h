#pragma once

#include "common/exception.h"
#include "common/concepts.h"

#include <algorithm>
#include <cmath>

namespace truck {

template<typename T>
inline T abs(const T& x) {
    return std::abs(x);
}

template<typename T>
inline double squared(const T& x) {
    return x * x;
}

template<typename T>
T clamp(const T& val, const T& low, const T& high) {
    return std::clamp(val, low, high);
}

template<typename T>
T clamp(const T& val, const T& abs_limit) {
    return clamp(val, -abs_limit, abs_limit);
}

template<typename T>
inline constexpr int sign(const T& x) {
    return (T(0) < x) - (x < T(0));
}

template<class T>
struct Limits {
    Limits() = default;

    Limits(const T& min, const T& max) : min(min), max(max) { VERIFY(min <= max); }

    bool isMet(const T& x) const { return min <= x && x <= max; }

    bool isStrictlyMet(const T& x) const { return min < x && x < max; }

    T clamp(const T& t) const { return truck::clamp(t, min, max); }

    double ratio(const T& t) const { return (clamp(t) - min) / (max - min); }

    T min, max;
};

template<class Int, class Float>
Int ceil(Float value) {
    return static_cast<Int>(std::ceil(value));
}

template<class Int, class Float>
Int floor(Float value) {
    return static_cast<Int>(std::floor(value));
}

template<class Int, class Float>
Int round(Float value) {
    return static_cast<Int>(std::round(value));
}

template<typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
inline constexpr size_t fls(T value) {
    size_t i = 0;
    for (; (value >> i) > 0; ++i) {
    }
    return i;
}

}  // namespace truck
