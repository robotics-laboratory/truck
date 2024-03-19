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

    // TODO - переписать на что-то более быстрое (мб дерево Ли-Чао или какая-то модификация
    // тернарного поиска)
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

    const auto longitude_step = params_.track_height / (params_.longitude_discretization - 1);
    const auto latitude_step = params_.track_width / (params_.latitude_discretization - 1);
    const auto forward_yaw_step = geom::Angle(M_PI / (params_.forward_yaw_discretization + 1));
    const auto backward_yaw_step = geom::Angle(M_PI / (params_.backward_yaw_discretization + 1));

    const auto dist_from_milestone = geom::distance(
        *nearest_segment_it,
        geom::projection(pose.pos, geom::Segment(*nearest_segment_it, *(nearest_segment_it + 1))));

    auto longitude_it =
        geom::UniformStepper(&route, longitude_step, dist_from_milestone, nearest_segment_it);
    longitude_it -= params_.track_height * params_.longitude_ratio;

    for (size_t i = 0; i < params_.longitude_discretization && longitude_it != route.uend();
         ++i, ++longitude_it) {
        const auto longitude_pose = *longitude_it;
        auto latitude_pose = geom::Pose(
            longitude_pose.pos +
                longitude_pose.dir.vec().right() * params_.track_width * params_.latitude_ratio,
            longitude_pose.dir);
        for (size_t j = 0; j < params_.latitude_discretization;
             ++j, latitude_pose.pos += latitude_pose.dir.vec().left() * latitude_step) {
            {
                auto state_pose = geom::Pose(
                    latitude_pose.pos, latitude_pose.dir.right().angle() + forward_yaw_step);
                for (size_t k = 0; k < params_.forward_yaw_discretization;
                     ++k, state_pose.dir += forward_yaw_step) {
                    state_poses_.push_back(state_pose);
                }
            }
            {
                auto state_pose = geom::Pose(
                    latitude_pose.pos, latitude_pose.dir.left().angle() + backward_yaw_step);
                for (size_t k = 0; k < params_.backward_yaw_discretization;
                     ++k, state_pose.dir += backward_yaw_step) {
                    state_poses_.push_back(state_pose);
                }
            }
        }
    }

    return *this;
}

const std::vector<geom::Pose>& Planner::GetStatePoses() const noexcept { return state_poses_; }

}  // namespace truck::trajectory_planner