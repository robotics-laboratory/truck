#pragma once

#include <boost/assert.hpp>

#include <algorithm>
#include <cmath>

namespace truck {

template<typename T>
inline double squared(const T& x) {
    return x * x;
}

template<typename T>
T clamp(const T& val, const T& low, const T& high) {
    return std::clamp(val, low, high);
}

template <typename T>
T clamp(const T& val, const T& abs_limit) {
    return clamp(val, -abs_limit, abs_limit);
}

template<class T>
struct Limits {
    Limits() = default;

    Limits(const T& min, const T& max) : min(min), max(max) { BOOST_VERIFY(min <= max); }

    bool isMet(const T& x) const { return min <= x && x <= max; }

    bool isStrictlyMet(const T& x) const { return min < x && x < max; }

    T clamp(const T& t) const { return truck::clamp(t, min, max); }

    T min, max;
};

template<class Int, class Float>
Int ceil(Float value) { return static_cast<Int>(std::ceil(value)); }

template<class Int, class Float>
Int floor(Float value) { return static_cast<Int>(std::floor(value)); }

template<class Int, class Float>
Int round(Float value) { return static_cast<Int>(std::round(value)); }

}  // namespace truck