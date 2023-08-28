#pragma once

#include "common/exception.h"

#include "fastgrid/grid.h"

#include "geom/pose.h"
#include "geom/transform.h"

namespace truck::fastgrid {

template<typename T>
class BilinearInterpolation {
  public:
    struct Domain {
        Domain(const double width, const double height, double resolution, const geom::Pose& origin)
            : width(width)
            , height(height)
            , resolution(resolution)
            , origin(origin)
            , tf(geom::Transform(origin.pos, origin.dir.unit()).inv()) {}

        geom::Vec2 Transform(const geom::Vec2& point) const { return tf(point); }

        std::pair<int, int> GetCell(const geom::Vec2& point) const {
            const geom::Vec2 rel_point = Transform(point);
            VERIFY(
                rel_point.x >= 0 && rel_point.x < width * resolution && rel_point.y >= 0 &&
                rel_point.y < height * resolution);
            return {
                static_cast<int>(rel_point.x / resolution),
                static_cast<int>(rel_point.y / resolution)};
        }

        double width = 0;
        double height = 0;
        double resolution = 0;

        geom::Pose origin;

        geom::Transform tf;
    };

    BilinearInterpolation(const Grid<T>& grid)
        : grid_(grid)
        , domain_(
              grid.size.width - 1, grid.size.height - 1, grid.resolution,
              geom::Pose(
                  grid.origin->pos + grid.origin->dir.unit() * grid.resolution / 2 +
                      grid.origin->dir.unit().left() * grid.resolution / 2,
                  grid.origin->dir)) {}

    double operator()(const geom::Vec2& point) const {
        const geom::Vec2 ref_point = domain_.Transform(point);
        const auto [row, col] = domain_.GetCell(point);
        const int index = row * grid_.size.width + col;
        return static_cast<double>(grid_.data[index]) *
                   ((row + 1) - ref_point.x / domain_.resolution) *
                   ((col + 1) - ref_point.y / domain_.resolution) +
               static_cast<double>(grid_.data[index + grid_.size.width]) *
                   (ref_point.x / domain_.resolution - row) *
                   ((col + 1) - ref_point.y / domain_.resolution) +
               static_cast<double>(grid_.data[index + 1]) *
                   ((row + 1) - ref_point.x / domain_.resolution) *
                   (ref_point.y / domain_.resolution - col) +
               static_cast<double>(grid_.data[index + grid_.size.width + 1]) *
                   (ref_point.x / domain_.resolution - row) *
                   (ref_point.y / domain_.resolution - col);
    }

  private:
    Grid<T> grid_;
    Domain domain_;
};

}  // namespace truck::fastgrid
