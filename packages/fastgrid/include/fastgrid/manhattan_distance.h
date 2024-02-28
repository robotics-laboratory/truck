#pragma once

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"
#include "geom/pose.h"

#include <vector>

namespace truck::fastgrid {

void manhattanDistance(
    const F32Grid& distance_transform, float eps, int* queue_buf, size_t total_sources,
    F32Grid& manhattan_distance);

void manhattanDistance(
    const F32Grid& distance_transform, const std::vector<geom::Vec2>& sources, float eps,
    int* queue_buf, F32Grid& manhattan_distance);

void manhattanDistance(
    const F32Grid& distance_transform, const std::vector<geom::Vec2>& sources, float eps,
    F32Grid& manhattan_distance);

F32GridHolder manhattanDistance(
    const F32Grid& distance_transform, const std::vector<geom::Vec2>& sources, float eps);

void manhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps, int* queue_buf,
    F32Grid& manhattan_distance);

void manhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps,
    F32Grid& manhattan_distance);

F32GridHolder manhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps);

}  // namespace truck::fastgrid
