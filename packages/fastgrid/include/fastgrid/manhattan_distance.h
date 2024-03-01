#pragma once

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"

#include "common/array_as_queue.h"

#include "geom/vector.h"

#include <vector>

namespace truck::fastgrid {

void manhattanDistance(
    const F32Grid& distance_transform, ArrayAsQueue<int>& queue, float eps,
    F32Grid& manhattan_distance);

F32GridHolder manhattanDistance(
    const F32Grid& distance_transform, ArrayAsQueue<int>& queue, float eps);

void manhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps, int* queue_buf,
    F32Grid& manhattan_distance);

void manhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps,
    F32Grid& manhattan_distance);

F32GridHolder manhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps);

}  // namespace truck::fastgrid
