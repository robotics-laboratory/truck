#pragma once

#include "geom/transform.h"
#include "geom/pose.h"
#include "common/exception.h"

#include <cstdint>
#include <optional>

namespace truck::fastgrid {

struct Size {
    constexpr int operator()() const noexcept { return width * height; }

    int width = 0;
    int height = 0;

    Size Extend(const int span) const noexcept { return {width + span, height + span}; }
};

template<typename T>
struct Grid {
    Grid() = default;

    Grid(
        const Size& size, const double resolution,
        const std::optional<geom::Pose>& origin = std::nullopt)
        : size(size)
        , resolution(resolution)
        , origin(origin)
        , tf(geom::Transform(origin->pos, origin->dir.unit()).inv()) {}

    void Reset(T* data) { this->data = data; }

    T* operator[](int row) noexcept { return data + row * size.width; }

    const T* operator[](int row) const noexcept { return data + row * size.width; }

    // for relative points
    std::pair<int, int> ToCell(const geom::Vec2& rel_point) const noexcept {
        return {
            static_cast<int>(rel_point.x / resolution), static_cast<int>(rel_point.y / resolution)};
    }

    int ToIndex(const geom::Vec2& rel_point) const noexcept {
        const auto [x, y] = ToCell(rel_point);
        return y * size.width + x;
    }

    // for global points
    std::optional<geom::Vec2> TryTransform(const geom::Vec2& point) const noexcept {
        if (!tf) {
            return std::nullopt;
        }
        return (*tf)(point);
    }

    std::optional<std::pair<int, int>> TryGetCell(const geom::Vec2& point) const noexcept {
        const auto rel_point = TryTransform(point);
        if (!rel_point || rel_point->x < 0 || rel_point->x >= size.width * resolution ||
            rel_point->y < 0 || rel_point->y >= size.height * resolution) {
            return std::nullopt;
        }
        return ToCell(*rel_point);
    }

    std::optional<int> TryGetIndex(const geom::Vec2& point) const noexcept {
        const auto cell = TryGetCell(point);
        if (!cell) {
            return std::nullopt;
        }
        return cell->second * size.width + cell->first;
    }

    geom::Vec2 Transform(const geom::Vec2& point) const {
        const auto rel_point = TryTransform(point);
        VERIFY(rel_point);
        return *rel_point;
    }

    std::pair<int, int> GetCell(const geom::Vec2& point) const {
        const auto cell = TryGetCell(point);
        VERIFY(cell);
        return *cell;
    }

    int GetIndex(const geom::Vec2& point) const {
        const auto index = TryGetIndex(point);
        VERIFY(index);
        return *index;
    }

    Size size = {};
    double resolution = 0.0;
    T* data = nullptr;
    std::optional<geom::Pose> origin = std::nullopt;
    std::optional<geom::Transform> tf = std::nullopt;
};

using U8Grid = Grid<uint8_t>;
using S32Grid = Grid<int32_t>;
using U32Grid = Grid<uint32_t>;
using F32Grid = Grid<float>;

}  // namespace truck::fastgrid
