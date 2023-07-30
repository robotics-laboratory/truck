#pragma once

#include "common/exception.h"
#include "fastgrid/grid.h"

#include <cmath>

namespace truck::fastgrid {

template<typename T>
class BilinearInterpolation {
  public:
    BilinearInterpolation(const Grid<T>& grid)
        : grid_(grid)
        , domain_(
              {.width = grid.size.width - 1, .height = grid.size.height - 1}, grid.resolution,
              geom::Pose(
                  grid.origin->pos + grid.origin->dir.unit() * grid.resolution / 2 +
                      grid.origin->dir.unit().left() * grid.resolution / 2,
                  grid.origin->dir)) {}

    double operator()(const geom::Vec2& point) const {
        const geom::Vec2 ref_point = domain_.GetReferencePoint(point);
        const auto [row, col] = domain_.GetReferenceCell(ref_point);
        return (static_cast<double>(grid_[row][col]) *
                    (domain_.resolution * (row + 1) - ref_point.x) *
                    (domain_.resolution * (col + 1) - ref_point.y) +
                static_cast<double>(grid_[row + 1][col]) *
                    (ref_point.x - domain_.resolution * row) *
                    (domain_.resolution * (col + 1) - ref_point.y) +
                static_cast<double>(grid_[row][col + 1]) *
                    (domain_.resolution * (row + 1) - ref_point.x) *
                    (ref_point.y - domain_.resolution * col) +
                static_cast<double>(grid_[row + 1][col + 1]) *
                    (ref_point.x - domain_.resolution * row) *
                    (ref_point.y - domain_.resolution * col)) /
               std::pow(domain_.resolution, 2);
    }

  private:
    Grid<T> grid_;
    Grid<T> domain_;
};

}  // namespace truck::fastgrid
