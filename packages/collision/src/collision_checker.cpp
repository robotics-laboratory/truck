#include "collision/collision_checker.h"

#include "common/exception.h"

#include <cstdint>

namespace truck::collision {

StaticCollisionChecker::StaticCollisionChecker(const model::Shape& shape) : shape_(shape) {}

bool StaticCollisionChecker::initialized() const { return state_.collision_map != nullptr; }

void StaticCollisionChecker::reset(std::shared_ptr<CollisionMap> collision_map) {
    state_.collision_map = std::move(collision_map);
}

double StaticCollisionChecker::distance(const geom::Vec2& point) const noexcept {
    VERIFY(initialized());

    auto interpolation = fastgrid::Bilinear<float>(state_.collision_map->GetDistanceMap());
    const auto [x, y] = interpolation.domain.Transform(point);

    // check borders
    if ((x < 0) || (y < 0) ||
        (interpolation.domain.size.height * interpolation.domain.resolution <= y) ||
        (interpolation.domain.size.width * interpolation.domain.resolution <= x)) {
        return kMaxDistance;
    }

    return interpolation.Get(point);
}

double StaticCollisionChecker::distance(const geom::Pose& ego_pose) const noexcept {
    double min_dist = kMaxDistance;
    std::vector<geom::Vec2> points = shape_.getCircleDecomposition(ego_pose);

    for (const geom::Vec2& point : points) {
        min_dist = std::min(min_dist, std::max(distance(point) - shape_.radius(), 0.0));
    }

    return min_dist;
}

}  // namespace truck::collision