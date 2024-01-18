#include "collision/collision_checker.h"

#include "common/exception.h"

#include <cstdint>

namespace truck::collision {

StaticCollisionChecker::StaticCollisionChecker(const model::Shape& shape) : shape_(shape) {}

bool StaticCollisionChecker::initialized() const { return state_.distance_transform.has_value(); }

void StaticCollisionChecker::reset(const fastgrid::F32Grid& distance_map) {
    state_.distance_transform = fastgrid::Bilinear<float>(distance_map);
}

double StaticCollisionChecker::distance(const geom::Vec2& point) const {
    VERIFY(initialized());
    const auto [x, y] = state_.distance_transform->domain.Transform(point);

    // check borders
    if ((x < 0) || (y < 0) ||
        (state_.distance_transform->domain.size.height *
             state_.distance_transform->domain.resolution <=
         y) ||
        (state_.distance_transform->domain.size.width *
             state_.distance_transform->domain.resolution <=
         x)) {
        return kMaxDistance;
    }

    return state_.distance_transform->Get(point);
}

double StaticCollisionChecker::distance(const geom::Pose& ego_pose) const {
    double min_dist = kMaxDistance;
    std::vector<geom::Vec2> points = shape_.getCircleDecomposition(ego_pose);

    for (const geom::Vec2& point : points) {
        min_dist = std::min(min_dist, std::max(distance(point) - shape_.radius(), 0.0));
    }

    return min_dist;
}

}  // namespace truck::collision