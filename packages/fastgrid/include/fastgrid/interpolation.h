#pragma once

#include "common/exception.h"

#include "fastgrid/grid.h"

#include "geom/pose.h"
#include "geom/transform.h"

namespace truck::fastgrid {

struct Domain {
    template<class T>
    Domain(const Grid<T>& grid) : size(grid.size), resolution(grid.resolution) {
        auto pose = VERIFY(grid.origin)->pose;
        const auto dx = grid.resolution / 2 * pose.dir;
        const auto dy = dx.left();
        pose.pos += dx + dy;

        origin = Origin(pose);
    }

    geom::Vec2 Transform(const geom::Vec2& point) const { return origin.tf(point); }

    Size size;
    double resolution;
    Origin origin;
};

template<typename T>
struct Bilinear {
    Bilinear(const Grid<T>& grid) : grid(grid), domain(grid) {}

    double GetRelative(const geom::Vec2& point) const noexcept {
        const double resolution = domain.resolution;
        const int x = static_cast<int>(point.x / domain.resolution);
        const int y = static_cast<int>(point.y / domain.resolution);

        const int index = y * domain.size.width + x;

        const double topLetf = grid.data[index];
        const double topRight = grid.data[index + 1];
        const double bottomLeft = grid.data[index + domain.size.width];
        const double bottomRight = grid.data[index + domain.size.width + 1];

        const double xRatio = (point.x / resolution - x);
        const double xRatioInv = 1 - xRatio;

        const double yRatio = (point.y / resolution - y);
        const double yRatioInv = 1 - yRatio;

        const double top = topLetf * xRatioInv + topRight * xRatio;
        const double bottom = bottomLeft * xRatioInv + bottomRight * xRatio;

        return top * yRatioInv + bottom * yRatio;
    }

    double Get(const geom::Vec2& point) const noexcept {
        const auto rel_point = domain.origin.tf(point);
        return GetRelative(rel_point);
    }

    double Get(const geom::Vec2& point, const T& default_value) const noexcept {
        const auto [x, y] = domain.origin.tf(point);
        if ((x < 0) || (y < 0) || (domain.size.width * domain.resolution <= x) ||
            (domain.size.height * domain.resolution <= y)) {
            return default_value;
        }
        return GetRelative({.x = x, .y = y});
    }

    double operator()(const geom::Vec2& point) const noexcept { return Get(point); }

    Grid<T> grid;
    Domain domain;
};

}  // namespace truck::fastgrid
