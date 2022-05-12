#pragma once

#include <cmath>
#include <type_traits>

namespace geom {

template <class T1, class T2>
[[ gnu::always_inline, nodiscard, gnu::pure ]] inline bool near(T1 a, T2 b,
                                                                double eps = 0) noexcept {
    if constexpr (std::is_floating_point_v<std::common_type_t<T1, T2>>) {
        return std::abs(a - b) <= eps;
    } else {
        static_assert(std::is_integral_v<T1> && std::is_integral_v<T2>,
                      "Values to be compared must have integral or floating point types");
        (void)eps;
        return a == b;
    }
}

}  // namespace geom
