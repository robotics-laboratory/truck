#pragma once

#include "geom/transform.h"
#include "geom/pose.h"
#include "common/exception.h"

#include <cstdint>
#include <optional>
#include <ostream>

namespace truck::fastgrid {

struct Size {
    constexpr int operator()() const noexcept { return width * height; }

    Size extend(int span) const noexcept { return {width + span, height + span}; }

    constexpr int max() const noexcept { return std::max(width, height); }

    int width = 0;
    int height = 0;
};

bool operator==(const Size& a, const Size& b) noexcept;
bool operator!=(const Size& a, const Size& b) noexcept;

std::ostream& operator<<(std::ostream& out, const Size& index) noexcept;

struct Index {
    int x = 0;
    int y = 0;
};

bool operator==(const Index& a, const Index& b) noexcept;
bool operator!=(const Index& a, const Index& b) noexcept;

std::ostream& operator<<(std::ostream& out, const Index& index) noexcept;

struct Origin {
    Origin() = default;

    Origin(const geom::Pose& pose) : pose(pose) { tf = geom::Transform(pose.pos, pose.dir).inv(); }

    geom::Pose pose{};
    geom::Transform tf{};
};

template<typename T>
struct Grid {
    Grid() = default;

    Grid(
        const Size& size, double resolution, const std::optional<geom::Pose>& origin = std::nullopt)
        : size(size), resolution(resolution) {
        if (origin) {
            this->origin = Origin(*origin);
        }
    }

    void reset(T* data) { this->data = data; }

    T* operator[](int row) noexcept { return data + row * size.width; }

    const T* operator[](int row) const noexcept { return data + row * size.width; }

    geom::Vec2 transform(const geom::Vec2& point) const noexcept {
        return (VERIFY(origin)->tf)(point);
    }

    // for relative points
    Index toIndex(int plain_index) const noexcept {
        return {plain_index % size.width, plain_index / size.width};
    }

    Index toIndex(const geom::Vec2& rel_point) const noexcept {
        return {
            static_cast<int>(rel_point.x / resolution), static_cast<int>(rel_point.y / resolution)};
    }

    int toPlainIndex(const geom::Vec2& rel_point) const noexcept {
        const auto [x, y] = toIndex(rel_point);
        return y * size.width + x;
    }

    std::optional<Index> tryGetIndex(const geom::Vec2& point) const noexcept {
        const auto index = toIndex(transform(point));
        if (0 <= index.x || index.x < size.width || 0 <= index.y || index.y < size.height) {
            return index;
        }

        return std::nullopt;
    }

    std::optional<int> tryGetPlainIndex(const geom::Vec2& point) const noexcept {
        const auto index = toIndex(transform(point));
        if (0 <= index.x || index.x < size.width || 0 <= index.y || index.y < size.height) {
            return index.y * size.width + index.x;
        }

        return std::nullopt;
    }

    Index getIndex(const geom::Vec2& point) const { return toIndex(transform(point)); }

    int getPlainIndex(const geom::Vec2& point) const { return toPlainIndex(transform(point)); }

    void SetTo(T t) noexcept { std::fill(data, data + size(), t); }

    Size size = {};
    double resolution = 0.0;
    T* data = nullptr;
    std::optional<Origin> origin = std::nullopt;
};

template<typename U, typename V>
bool equal(const Grid<U>& a, const Grid<V>& b, double eps) noexcept {
    // add eps to compare resolution?
    if (a.size != b.size || a.resolution != b.resolution) {
        return false;
    }

    std::equal(a.data, a.data + a.size(), b.data, [eps](const U& a, const V& b) {
        return std::abs(a - b) < eps;
    });

    return true;
}

using U8Grid = Grid<uint8_t>;
using S32Grid = Grid<int32_t>;
using U32Grid = Grid<uint32_t>;
using F32Grid = Grid<float>;

}  // namespace truck::fastgrid
