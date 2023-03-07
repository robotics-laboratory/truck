#include "collision_checker/collision_checker.h"

#include <cstdint>

namespace truck::collision_checker {

StaticCollisionChecker::StaticCollisionChecker(const model::Shape& shape) : shape_(shape) {}

bool StaticCollisionChecker::initialized() const { return state_.has_value(); }

void StaticCollisionChecker::reset(const MapMeta& meta, const cv::Mat& distance_transform) {
    state_ = {
        .meta = meta,
        .tf = geom::Transform(meta.origin.pos, meta.origin.dir),
        .distance_transform = distance_transform
    };
}

double StaticCollisionChecker::distance(const geom::Vec2& point) const {
    BOOST_ASSERT(initialized());
    const auto grid_point = state_->tf(point);

    // find relevant indices of distance transform matrix
    const auto width_index = floor<int>(grid_point.x / state_->meta.resolution);
    const auto height_index = floor<int>(grid_point.y / state_->meta.resolution);

    // check borders
    if ((width_index >= static_cast<int>(state_->meta.width)) ||
        (height_index >= static_cast<int>(state_->meta.height)) || (width_index < 0) ||
        (height_index < 0)) {
        return kMaxDistance;
    }

    const double distance =
        state_->distance_transform.at<float>(height_index, width_index) * state_->meta.resolution;

    return std::max(distance - shape_.radius(), 0.0);
}

double StaticCollisionChecker::distance(const geom::Pose& ego_pose) const {
    double min_dist = kMaxDistance;
    std::vector<geom::Vec2> points = shape_.getCircleDecomposition(ego_pose);

    for (const geom::Vec2& point : points) {
        min_dist = std::min(min_dist, distance(point));
    }

    return min_dist;
}

}  // namespace truck::collision_checker