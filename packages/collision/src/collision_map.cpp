#include "collision/collision_map.h"

#include "common/math.h"

#include "fastgrid/distance_transform.h"
#include "fastgrid/draw.h"

#include "geom/msg.h"

#include <opencv2/imgproc.hpp>

#include <vector>

namespace truck::collision {

CollisionMap& CollisionMap::SetOccupancyGrid(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid) noexcept {
    state_.occupancy_grid = std::move(occupancy_grid);
    bit_map_.is_consistent = false;
    distance_map_.is_consistent = false;
    return *this;
}

CollisionMap& CollisionMap::SetMap(std::shared_ptr<const map::Map> map) noexcept {
    state_.map = std::move(map);
    bit_map_.is_consistent = false;
    distance_map_.is_consistent = false;
    return *this;
}

fastgrid::U8Grid CollisionMap::GetBitMap() noexcept {
    if (!state_.occupancy_grid) {
        return fastgrid::U8Grid(fastgrid::Size{.width = 0, .height = 0}, 0.0);
    }

    if (bit_map_.is_consistent) {
        return bit_map_.holder.grid;
    }

    auto occupancy_grid_size = fastgrid::Size{
        .width = static_cast<int>(state_.occupancy_grid->info.width),
        .height = static_cast<int>(state_.occupancy_grid->info.height)};

    if (bit_map_.holder.grid.size != occupancy_grid_size) {
        bit_map_.holder = fastgrid::makeGrid<uint8_t>(
            occupancy_grid_size, state_.occupancy_grid->info.resolution);
    }

    auto& grid = bit_map_.holder.grid;

    grid.resolution = state_.occupancy_grid->info.resolution;
    grid.origin = geom::toPose(state_.occupancy_grid->info.origin);

    for (int i = 0; i < occupancy_grid_size.height; ++i) {
        for (int j = 0; j < occupancy_grid_size.width; ++j) {
            int8_t grid_cell = state_.occupancy_grid->data.at(i * occupancy_grid_size.width + j);
            grid.data[i * grid.size.width + j] = (grid_cell == 0);
        }
    }

    if (state_.map) {
        for (const auto& polygon : state_.map->polygons()) {
            fastgrid::Draw(polygon, grid);
        }
    }

    bit_map_.is_consistent = true;

    return grid;
}

fastgrid::F32Grid CollisionMap::GetDistanceMap() noexcept {
    if (!state_.occupancy_grid) {
        return fastgrid::F32Grid(fastgrid::Size{.width = 0, .height = 0}, 0.0);
    }

    if (distance_map_.is_consistent) {
        return distance_map_.holder.grid;
    }

    auto bit_map_grid = GetBitMap();

    if (distance_map_.holder.grid.size != bit_map_grid.size) {
        distance_map_.holder = fastgrid::makeGridLike<float>(bit_map_grid);
        distance_map_.buffer =
            fastgrid::makeGrid<int>(bit_map_grid.size.extend(2), bit_map_grid.resolution);
    }

    auto& grid = distance_map_.holder.grid;

    grid.resolution = bit_map_grid.resolution;
    grid.origin = bit_map_grid.origin;

    fastgrid::distanceTransformApprox3(bit_map_grid, distance_map_.buffer.grid, grid);

    bit_map_.is_consistent = true;

    return grid;
}

nav_msgs::msg::OccupancyGrid CollisionMap::MakeCostMap(
    const std_msgs::msg::Header& header, double kMaxDist) noexcept {
    auto grid = GetDistanceMap();

    nav_msgs::msg::OccupancyGrid msg;

    msg.header = header;
    msg.info.resolution = grid.resolution;
    msg.info.width = grid.size.width;
    msg.info.height = grid.size.height;
    msg.info.origin = geom::msg::toPose(grid.origin->pose);
    msg.data = std::vector<int8_t>(grid.size());

    for (int i = 0; i < grid.size.height; ++i) {
        for (int j = 0; j < grid.size.width; ++j) {
            msg.data.at(i * grid.size.width + j) = static_cast<int8_t>(
                100 * (1 - Limits(0., 1.).clamp(grid.data[i * grid.size.width + j] / kMaxDist)));
        }
    }

    return msg;
}

}  // namespace truck::collision
