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
};

template<typename T>
struct Grid {
    Grid() = default;

    Grid(
        const Size& size, double resolution, const std::optional<geom::Pose>& origin = std::nullopt)
        : size(size)
        , resolution(resolution)
        , origin(origin)
        , tf(geom::Transform(origin->pos, origin->dir.unit()).inv()) {}

    void Reset(T* data) { this->data = data; }

    T* operator[](int row) noexcept { return data + row * size.width; }

    const T* operator[](int row) const noexcept { return data + row * size.width; }

    geom::Vec2 GetRelativePoint(const geom::Vec2& point) const {
        VERIFY(tf);
        return (*tf)(point);
    }

    bool VerifyRelativePoint(const geom::Vec2& ref_point) const {
        return ref_point.x >= 0 && ref_point.x < size.width * resolution && ref_point.y >= 0 &&
               ref_point.y < size.height * resolution;
    }

    bool VerifyPoint(const geom::Vec2& point) const {
        const geom::Vec2 ref_point = GetRelativePoint(point);
        return VerifyRelativePoint(ref_point);
    }

    std::pair<int, int> GetRelativeCell(const geom::Vec2& ref_point) const {
        VERIFY(VerifyRelativePoint(ref_point));
        return {
            static_cast<int>(ref_point.x / resolution), static_cast<int>(ref_point.y / resolution)};
    }

    std::pair<int, int> GetCell(const geom::Vec2& point) const {
        return GetRelativeCell(GetRelativePoint(point));
    }

    int GetRelativeIndex(const geom::Vec2& point) const {
        std::pair<int, int> cell = GetRelativeCell(point);
        return cell.second * size.width + cell.first;
    }

    int GetIndex(const geom::Vec2& point) const {
        return GetRelativeIndex(GetRelativePoint(point));
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
