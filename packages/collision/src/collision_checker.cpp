#include "collision/collision_checker.h"

#include "common/exception.h"

#include <cstdint>

namespace truck::collision {

StaticCollisionChecker::StaticCollisionChecker(const model::Shape& shape) : shape_(shape) {}

bool StaticCollisionChecker::initialized() const { return state_.has_value(); }

void StaticCollisionChecker::reset(Map distance_transform) {
    const auto& origin = distance_transform.origin;
    const auto tf = geom::Transform(origin.pos, origin.dir);

    state_ = State{
        .distance_transform = std::move(distance_transform),
        .tf = tf.inv(),
    };
}

double StaticCollisionChecker::distance(const geom::Vec2& point) const {
    VERIFY(initialized());
    const auto grid_point = state_->tf(point);

    // find relevant indices of distance transform matrix
    const auto x = floor<int>(grid_point.x / state_->distance_transform.resolution);
    const auto y = floor<int>(grid_point.y / state_->distance_transform.resolution);

    // check borders
    if ((x < 0) || (y < 0) || (state_->distance_transform.size.height <= y)
        || (state_->distance_transform.size.width <= x)) {
        return kMaxDistance;
    }

    return state_->distance_transform.data.at<float>(y, x) * state_->distance_transform.resolution;
}

double StaticCollisionChecker::distance(const geom::Pose& ego_pose) const {
    double min_dist = kMaxDistance;
    const std::vector<geom::Vec2> points = shape_.getCircleDecomposition(ego_pose);

    for (const geom::Vec2& point : points) {
        min_dist = std::min(min_dist, std::max(distance(point) - shape_.radius(), 0.0));
    }

    return min_dist;
}

}  // namespace truck::collision
