#include "trajectory_planner/planner.h"

#include "common/math.h"

#include "geom/distance.h"
#include "geom/segment.h"
#include "geom/uniform_stepper.h"

#include <algorithm>

namespace truck::trajectory_planner {

Planner::Planner(const Params& params) : params_(params) {}

Planner& Planner::Build(const geom::Pose& pose, const geom::Polyline& route) {
    state_poses_.clear();

    // TODO - подумать, можно ли быстрее
    auto nearest_segment_it = route.begin();
    auto nearest_segement_distance_sq =
        geom::distanceSq(pose.pos, geom::Segment(*nearest_segment_it, *(nearest_segment_it + 1)));
    for (auto it = route.begin(); it + 1 != route.end(); ++it) {
        const auto distance = geom::distanceSq(pose.pos, geom::Segment(*it, *(it + 1)));
        if (distance < nearest_segement_distance_sq) {
            nearest_segment_it = it;
            nearest_segement_distance_sq = distance;
        }
    }

    const auto dist_from_milestone = geom::distance(
        *nearest_segment_it,
        geom::projection(pose.pos, geom::Segment(*nearest_segment_it, *(nearest_segment_it + 1))));

    auto longitude_it = geom::UniformStepper(
        &route, params_.longitude.Step(), dist_from_milestone, nearest_segment_it);
    longitude_it += params_.longitude.limits.min;

    for (size_t i = 0; i < params_.longitude.total_states && longitude_it != route.uend();
         ++i, ++longitude_it) {
        const auto longitude_pose = *longitude_it;
        for (size_t j = 0; j < params_.latitude.total_states; ++j) {
            const auto latitude_pose = geom::Pose(
                longitude_pose.pos + longitude_pose.dir.vec().left() * params_.latitude[j],
                longitude_pose.dir);

            for (size_t k = 0; k < params_.forward_yaw.total_states; ++k) {
                state_poses_.push_back(geom::Pose(
                    latitude_pose.pos,
                    latitude_pose.dir.angle() + geom::Angle(params_.forward_yaw[k])));
            }
            for (size_t k = 0; k < params_.backward_yaw.total_states; ++k) {
                state_poses_.push_back(geom::Pose(
                    latitude_pose.pos,
                    latitude_pose.dir.angle() + geom::Angle(params_.backward_yaw[k])));
            }
        }
    }

    return *this;
}

const std::vector<geom::Pose>& Planner::GetStatePoses() const noexcept { return state_poses_; }

}  // namespace truck::trajectory_planner