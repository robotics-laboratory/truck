#include "fastgrid/distance_transform.h"
#include "fastgrid/misc.h"

#include <array>
#include <limits>

namespace truck::fastgrid {

// Better to move to common
#define ALWAYS_INLINE inline __attribute__((always_inline))

namespace impl {

struct Step {
    int32_t hv, dg, ldg;
};

constexpr Step kStep3 = {95, 137, 0};
constexpr Step kStep5 = {10, 14, 22};

ALWAYS_INLINE void intDistToFloat(
    const int32_t* data, int size, float scale, float* out_it) noexcept {
    for (const auto* it = data; it != data + size; ++it, ++out_it) {
        *out_it = scale * (*it);
    }
}

template<Step step>
ALWAYS_INLINE int32_t corner3(const int32_t* lane, int32_t side) noexcept {
    const int32_t result = std::min(side, lane[1]) + step.hv;
    const int32_t tmp = std::min(lane[0], lane[2]) + step.dg;

    if (tmp < result) {
        return tmp;
    }

    return result;
}

template<Step step>
ALWAYS_INLINE int32_t corner5(const int32_t* first, const int32_t* second, int32_t side) noexcept {
    int32_t result = std::min(side, first[2]) + step.hv;
    int32_t tmp = std::min(first[1], first[3]) + step.dg;

    if (tmp < result) {
        result = tmp;
    }

    tmp = step.ldg + std::min(std::min(first[0], first[4]), std::min(second[1], second[3]));

    if (tmp < result) {
        return tmp;
    }

    return result;
}

template<int border, Step step>
ALWAYS_INLINE int32_t topLeftCorner(const int32_t* buf_it, const Size& buf_size) noexcept {
    static_assert(border == 1 || border == 2, "Unexpected border!");

    if constexpr (border == 1) {
        const int32_t* lane = buf_it - buf_size.width - border;
        return corner3<step>(lane, *(buf_it - 1));
    }

    if constexpr (border == 2) {
        const int32_t* first = buf_it - buf_size.width - border;
        const int32_t* second = first - buf_size.width;
        return corner5<step>(first, second, *(buf_it - 1));
    }

    return {};
}

template<int border, Step step>
ALWAYS_INLINE int32_t bottomRightCorner(const int32_t* buf_it, const Size& buf_size) noexcept {
    static_assert(border == 1 || border == 2, "Unexpected border!");

    if constexpr (border == 1) {
        const int32_t* lane = buf_it + buf_size.width - border;
        return corner3<step>(lane, *(buf_it + 1));
    }

    if constexpr (border == 2) {
        const int32_t* first = buf_it + buf_size.width - border;
        const int32_t* second = first + buf_size.width;
        return corner5<step>(first, second, *(buf_it + 1));
    }

    return {};
}

template<int border, Step step>
ALWAYS_INLINE void distanceTransformApprox(
    const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept {
    VERIFY(buf.size == input.size.extend(2 * border));
    VERIFY(out.size == input.size);

    const auto scale = static_cast<float>(input.resolution) / step.hv;
    const auto default_value = step.dg * input.size.max() / 6;

    // fill top and bottom rows
    for (int i = 0; i < border; ++i) {
        auto* top = buf.data + i * buf.size.width;
        setTo(top, buf.size.width, default_value);

        auto* bottom = buf.data + (buf.size.height - 1 - i) * buf.size.width;
        setTo(bottom, buf.size.width, default_value);
    }

    // forward scan
    for (int i = 0; i < input.size.height; ++i) {
        auto* buf_it = buf.data + (border + i) * buf.size.width + border;

        // fill left side
        for (int j = 1; j <= border; ++j) {
            buf_it[-j] = default_value;
        }

        // fill right side
        for (int j = 0; j < border; ++j) {
            buf_it[out.size.width + j] = default_value;
        }

        const auto* it = input.data + i * input.size.width;
        const auto* end = it + input.size.width;

        for (; it != end; ++it, ++buf_it) {
            if (*it != 0) {
                *buf_it = 0;
                continue;
            }

            *buf_it = topLeftCorner<border, step>(buf_it, buf.size);
        }
    }

    // backward scan
    for (int i = out.size.height - 1; i >= 0; --i) {
        auto* buf_row = buf.data + (i + border) * buf.size.width + border;

        auto* end = buf_row - 1;
        for (auto it = end + out.size.width; it != end; --it) {
            if (*it <= step.hv) {
                continue;
            }

            *it = std::min(*it, bottomRightCorner<border, step>(it, buf.size));
        }

        float* out_it = out.data + i * out.size.width;
        intDistToFloat(buf_row, out.size.width, scale, out_it);
    }

    out.resolution = input.resolution;
    out.origin = input.origin;
}

template<int border, Step step>
ALWAYS_INLINE void distanceTransformApprox(const U8Grid& input, F32Grid& out) noexcept {
    auto buf = makeGrid<int32_t>(input.size.extend(2 * border), input.resolution);
    distanceTransformApprox<border, step>(input, *buf, out);
}

template<int border, Step step>
ALWAYS_INLINE F32GridHolder distanceTransformApprox(const U8Grid& input) noexcept {
    auto buf = makeGrid<int32_t>(input.size.extend(2 * border), input.resolution);
    auto out = makeGridLike<float>(input);

    distanceTransformApprox<border, step>(input, *buf, *out);
    return out;
}

}  // namespace impl

/* DistanceTransformApprox3 */

void distanceTransformApprox3(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept {
    impl::distanceTransformApprox<1, impl::kStep3>(input, buf, out);
}

void distanceTransformApprox3(const U8Grid& input, F32Grid& out) noexcept {
    impl::distanceTransformApprox<1, impl::kStep3>(input, out);
}

F32GridHolder distanceTransformApprox3(const U8Grid& input) noexcept {
    return impl::distanceTransformApprox<1, impl::kStep3>(input);
}

/* DistanceTransformApprox5 */

void distanceTransformApprox5(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept {
    impl::distanceTransformApprox<2, impl::kStep5>(input, buf, out);
}

void distanceTransformApprox5(const U8Grid& input, F32Grid& out) noexcept {
    impl::distanceTransformApprox<2, impl::kStep5>(input, out);
}

F32GridHolder distanceTransformApprox5(const U8Grid& input) noexcept {
    return impl::distanceTransformApprox<2, impl::kStep5>(input);
}

}  // namespace truck::fastgrid
