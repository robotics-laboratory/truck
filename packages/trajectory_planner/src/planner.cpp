#include "trajectory_planner/planner.h"

#include "common/math.h"

#include "geom/distance.h"
#include "geom/segment.h"
#include "geom/uniform_stepper.h"

namespace truck::trajectory_planner {

Planner::Planner(const Params& params) : params_(params) {}

Planner& Planner::Build(const geom::Pose& pose, const geom::Polyline& route) {
    state_poses_.clear();

    // TODO - переписать на что-то более быстрое (например, дерево Ли-Чао или какая-то модификация
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

    const auto dist_from_milestone =
        std::sqrt((pose.pos - *nearest_segment_it).lenSq() - nearest_segement_distance_sq);
    const auto longitude_step_length =
        params_.track_height / (params_.longitude_discretization - 1);
    const auto latitude_step_length = params_.track_width / (params_.latitude_discretization - 1);
    auto longitude_it = geom::UniformStepper(
        &route, longitude_step_length, dist_from_milestone, nearest_segment_it);
    longitude_it -= params_.track_height * params_.longitude_ratio;

    for (size_t i = 0; i < params_.longitude_discretization && longitude_it != route.uend();
         ++i, ++longitude_it) {
        auto state_pose = geom::Pose(
            (*longitude_it).pos +
                (*longitude_it).dir.vec().right().unit() * params_.track_width * 0.5,
            (*longitude_it).dir);
        for (size_t j = 0; j < params_.latitude_discretization;
             ++j, state_pose.pos += state_pose.dir.vec().left() * latitude_step_length) {
            const auto forward_yaw_step =
                geom::AngleVec2(geom::Angle(M_PI / (params_.forward_yaw_discretization + 1)));
            auto pose = geom::Pose(state_pose.pos, state_pose.dir.right() + forward_yaw_step);
            for (size_t k = 0; k < params_.forward_yaw_discretization;
                 ++k, pose.dir += forward_yaw_step) {
                state_poses_.push_back(pose);
            }
            const auto backward_yaw_step =
                geom::AngleVec2(geom::Angle(M_PI / (params_.backward_yaw_discretization + 1)));
            pose = geom::Pose(state_pose.pos, state_pose.dir.left() + backward_yaw_step);
            for (size_t k = 0; k < params_.backward_yaw_discretization;
                 ++k, pose.dir += backward_yaw_step) {
                state_poses_.push_back(pose);
            }
        }
    }

    return *this;
}

const std::vector<geom::Pose>& Planner::GetStatePoses() const noexcept { return state_poses_; }

}  // namespace truck::trajectory_planner