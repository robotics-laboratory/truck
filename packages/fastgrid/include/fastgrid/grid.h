#pragma once

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
        : size(size), resolution(resolution), origin(origin) {}

    void Reset(T* data) { this->data = data; }

    T* operator[](int row) noexcept { return data + row * size.width; }

    const T* operator[](int row) const noexcept { return data + row * size.width; }

    geom::Vec2 GetReferencePoint(const geom::Vec2& point) const {
        VERIFY(origin);
        return (point - origin->pos).rotate(origin->dir.inv().unit());
    }

    bool VerifyReferencePoint(const geom::Vec2& ref_point) const {
        return ref_point.x >= 0 && ref_point.x < size.width * resolution && ref_point.y >= 0 &&
               ref_point.y < size.height * resolution;
    }

    bool VerifyPoint(const geom::Vec2& point) const {
        const geom::Vec2 ref_point = GetReferencePoint(point);
        return VerifyReferencePoint(ref_point);
    }

    std::pair<int, int> GetReferenceCell(const geom::Vec2& ref_point) const {
        VERIFY(VerifyReferencePoint(ref_point));
        return {
            static_cast<int>(ref_point.x / resolution), static_cast<int>(ref_point.y / resolution)};
    }

    std::pair<int, int> GetCell(const geom::Vec2& point) const {
        return GetReferenceCell(GetReferencePoint(point));
    }

    int GetReferenceIndex(const geom::Vec2& point) const {
        std::pair<int, int> cell = GetReferenceCell(point);
        return cell.second * size.width + cell.first;
    }

    int GetIndex(const geom::Vec2& point) const {
        return GetReferenceIndex(GetReferencePoint(point));
    }

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
