#pragma once

#include "geom/pose.h"

#include <cstdint>
#include <optional>

namespace truck::fastgrid {

struct Size {
    constexpr size_t operator()() const noexcept { return width * height; }

    size_t width = 0;
    size_t height = 0;
};

template<typename T>
struct Grid {
    Grid() = default;

    Grid(
        const Size& size, double resolution, const std::optional<geom::Pose>& origin = std::nullopt)
        : size(size), resolution(resolution), origin(origin) {}

    void Reset(T* data) { this->data = data; }

    Size size = {};
    double resolution = 0.0;
    T* data = nullptr;
    std::optional<geom::Pose> origin = std::nullopt;
};

using U8Grid = Grid<uint8_t>;
using S32Grid = Grid<int32_t>;
using U32Grid = Grid<uint32_t>;
using F32Grid = Grid<float>;

}  // namespace truck::fastgrid