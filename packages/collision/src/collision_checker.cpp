#include "collision/collision_checker.h"

#include "common/exception.h"
#include "common/math.h"

#include "fastgrid/distance_transform.h"
#include "fastgrid/draw.h"
#include "fastgrid/interpolation.h"

#include "geom/msg.h"

#include <cstdint>
#include <vector>

namespace truck::collision {

StaticCollisionChecker::StaticCollisionChecker(const Params& params, const model::Shape& shape)
    : params_(params)
    , shape_(shape)
    , bit_map_cache_{
          .bit_map = fastgrid::makeGrid<uint8_t>(
              fastgrid::Size{
                  .width = 2 * ceil<int>(params_.radius / params_.resolution) + 1,
                  .height = 2 * ceil<int>(params_.radius / params_.resolution) + 1},
              params_.resolution),
          .is_consistent = true}
    , distance_map_cache_{
          .buffer = fastgrid::makeGrid<int>(
              fastgrid::Size{
                  .width = 2 * ceil<int>(params_.radius / params_.resolution) + 1,
                  .height = 2 * ceil<int>(params_.radius / params_.resolution) + 1}
                  .extend(2),
              params_.resolution),
          .distance_map = fastgrid::makeGrid<float>(
              fastgrid::Size{
                  .width = 2 * ceil<int>(params_.radius / params_.resolution) + 1,
                  .height = 2 * ceil<int>(params_.radius / params_.resolution) + 1},
              params_.resolution),
          .is_consistent = true} {}

bool StaticCollisionChecker::Initialized() const noexcept { return state_.origin != std::nullopt; }

StaticCollisionChecker& StaticCollisionChecker::SetEgoPose(const geom::Pose& ego_pose) noexcept {
    state_.origin = geom::Pose(
        ego_pose.pos - ego_pose.dir * params_.radius - ego_pose.dir.right() * params_.radius,
        ego_pose.dir.right());
    ResetCache();
    return *this;
}

StaticCollisionChecker& StaticCollisionChecker::SetOccupancyGrid(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid) noexcept {
    state_.occupancy_grid = std::move(occupancy_grid);
    ResetCache();
    return *this;
}

StaticCollisionChecker& StaticCollisionChecker::SetMap(
    std::shared_ptr<const map::Map> map) noexcept {
    state_.map = std::move(map);
    ResetCache();
    return *this;
}

fastgrid::U8Grid StaticCollisionChecker::BitMap() const noexcept {
    if (!bit_map_cache_.is_consistent) {
        VERIFY(state_.origin);

        auto& grid = bit_map_cache_.bit_map.grid;
        grid.origin = *state_.origin;
        grid.SetTo(0);

        if (state_.occupancy_grid) {
            const auto origin = geom::toPose(state_.occupancy_grid->info.origin);
            const auto resolution = state_.occupancy_grid->info.resolution;

            for (int i = 0; i < state_.occupancy_grid->info.height; ++i) {
                for (int j = 0; j < state_.occupancy_grid->info.width; ++j) {
                    const auto value =
                        state_.occupancy_grid->data.at(i * state_.occupancy_grid->info.width + j);
                    if (value == 0) {
                        continue;
                    }
                    const auto cell_origin = geom::Pose(
                        origin.pos + origin.dir.left() * i * resolution +
                            origin.dir * j * resolution,
                        origin.dir);
                    fastgrid::Draw(
                        geom::Polygon{
                            cell_origin.pos,
                            cell_origin.pos + cell_origin.dir.left() * resolution,
                            cell_origin.pos + cell_origin.dir * resolution +
                                cell_origin.dir.left() * resolution,
                            cell_origin.pos + cell_origin.dir * resolution},
                        grid,
                        value);
                }
            }
        }

        if (state_.map) {
            for (const auto& polygon : state_.map->polygons()) {
                fastgrid::Draw(polygon, grid, 100);
            }
        }

        bit_map_cache_.is_consistent = true;
    }

    return bit_map_cache_.bit_map.grid;
}

fastgrid::F32Grid StaticCollisionChecker::DistanceMap() const noexcept {
    if (!distance_map_cache_.is_consistent) {
        auto bit_map_grid = BitMap();
        auto& grid = distance_map_cache_.distance_map.grid;
        grid.origin = bit_map_grid.origin;
        fastgrid::distanceTransformApprox3(bit_map_grid, distance_map_cache_.buffer.grid, grid);

        distance_map_cache_.is_consistent = true;
    }

    return distance_map_cache_.distance_map.grid;
}

nav_msgs::msg::OccupancyGrid StaticCollisionChecker::MakeCostMap(
    const std_msgs::msg::Header& header, double kMaxDist) const noexcept {
    auto grid = DistanceMap();

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

double StaticCollisionChecker::Distance(const geom::Vec2& point) const noexcept {
    VERIFY(Initialized());

    auto interpolation = fastgrid::Bilinear<float>(DistanceMap());

    return interpolation.Get(point, kMaxDistance);
}

double StaticCollisionChecker::Distance(const geom::Pose& ego_pose) const noexcept {
    double min_dist = kMaxDistance;
    std::vector<geom::Vec2> points = shape_.getCircleDecomposition(ego_pose);

    for (const geom::Vec2& point : points) {
        min_dist = std::min(min_dist, std::max(Distance(point) - shape_.radius(), 0.0));
    }

    return min_dist;
}

void StaticCollisionChecker::ResetCache() noexcept {
    bit_map_cache_.is_consistent = false;
    distance_map_cache_.is_consistent = false;
}

}  // namespace truck::collision