#pragma once

#include <boost/assert.hpp>

#include <algorithm>

namespace truck {

template <typename T>
inline double squared(const T& x) {
    return x * x;
}

template<typename T>
T clamp(const T& val, const T& low, const T& high) {
    return std::min(std::max(val, low), high);
}

template <typename T>
T clamp(const T& val, const T& abs_limit) {
    return clamp(val, -abs_limit, abs_limit);
}

template<class T>
struct Limits { 
    Limits() = default;

    Limits(const T& min, const T& max) : min(min), max(max) { BOOST_VERIFY(min <= max); }

    inline  bool isMet(const T& x) const { return min <= x && x <= max; }

    inline  bool isStrictlyMet(const T& x) const { return min < x && x < max; }

    T clamp(const T& t) const { return std::clamp(t, min, max); }

    T min, max;
};

} // namespace truck