#pragma once

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"
#include "geom/pose.h"

namespace truck::fastgrid {

void ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps,
    std::vector<int>& queue_buf, F32Grid& manhattan_distance);

void ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps,
    F32Grid& manhattan_distance);

F32GridHolder ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps);

}  // namespace truck::fastgrid
