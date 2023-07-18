#pragma once

#include "geom/common.h"
#include "fastgrid/grid.h"

#include <cmath>
#include <exception>

namespace truck::fastgrid {

template<typename T>
class BilinearInterpolation {
  public:
    BilinearInterpolation(const Grid<T>& grid) : grid_(grid) {}

    double operator()(const geom::Vec2& point) const {
        if (point.x < 0 || point.y < 0 || point.x >= (grid_.size.width - 1) * grid_.resolution ||
            point.y >= (grid_.size.height - 1) * grid_.resolution) {
            throw std::domain_error("Point is not inside the grid");
        }
        const int col = static_cast<int>(point.x / grid_.resolution);
        const int row = static_cast<int>(point.y / grid_.resolution);
        return (static_cast<double>(grid_[row][col]) * (grid_.resolution * (row + 1) - point.x) *
                    (grid_.resolution * (col + 1) - point.y) +
                static_cast<double>(grid_[row + 1][col]) * (point.x - grid_.resolution * row) *
                    (grid_.resolution * (col + 1) - point.y) +
                static_cast<double>(grid_[row][col + 1]) *
                    (grid_.resolution * (row + 1) - point.x) * (point.y - grid_.resolution * col) +
                static_cast<double>(grid_[row + 1][col + 1]) * (point.x - grid_.resolution * row) *
                    (point.y - grid_.resolution * col)) /
               std::pow(grid_.resolution, 2);
    }

  private:
    Grid<T> grid_{};
};

}  // namespace truck::fastgrid
