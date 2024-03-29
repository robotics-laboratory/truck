#include "trajectory_planner/state.h"

#include "geom/distance.h"

namespace truck::trajectory_planner {

bool StateArea::IsInside(const State& state) const noexcept {
    return Limits<double>(x_range.min + base_state.pose.pos.x, x_range.max + base_state.pose.pos.x)
               .isMet(state.pose.pos.x) &&
           Limits<double>(y_range.min + base_state.pose.pos.y, y_range.max + base_state.pose.pos.y)
               .isMet(state.pose.pos.y) &&
           Limits<geom::Angle>(
               yaw_range.min + base_state.pose.dir.angle(),
               yaw_range.max + base_state.pose.dir.angle())
               .isMet(state.pose.dir.angle()) &&
           Limits<double>(
               velocity_range.min + base_state.velocity, velocity_range.max + base_state.velocity)
               .isMet(state.velocity);
}

StateSpace::StateSpace(const Params& params) : params_(params) {}

StateSpace::StateSpace(const StateSpace& other) : params_(other.params_) {
    states_.reserve(other.states_.size());
    for (const auto& state : other.states_) {
        states_.push_back(state);
        if (other.finish_states_.contains(&state)) {
            finish_states_.insert(&states_.back());
        }
    }
}

StateSpace& StateSpace::operator=(StateSpace other) & {
    std::swap(params_, other.params_);
    std::swap(states_, other.states_);
    std::swap(finish_states_, other.finish_states_);
    return *this;
}

StateSpace& StateSpace::Build(
    const State& ego_state, const StateArea& finish_area, const geom::Polyline& route) noexcept {
    Clear();

    states_.reserve(
        1 + params_.longitude.total_states * params_.latitude.total_states *
                (params_.forward_yaw.total_states + params_.backward_yaw.total_states) *
                params_.velocity.total_states);

    states_.push_back(ego_state);
    const auto start_pose = ego_state.pose;

    // TODO - подумать, можно ли быстрее
    auto nearest_segment_it = route.begin();
    auto nearest_segement_distance_sq = geom::distanceSq(
        start_pose.pos, geom::Segment(*nearest_segment_it, *(nearest_segment_it + 1)));
    for (auto it = route.begin(); it + 1 != route.end(); ++it) {
        const auto distance = geom::distanceSq(start_pose.pos, geom::Segment(*it, *(it + 1)));
        if (distance < nearest_segement_distance_sq) {
            nearest_segment_it = it;
            nearest_segement_distance_sq = distance;
        }
    }

    const auto dist_from_milestone = geom::distance(
        *nearest_segment_it,
        geom::projection(
            start_pose.pos, geom::Segment(*nearest_segment_it, *(nearest_segment_it + 1))));

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
                for (size_t v = 0; v < params_.velocity.total_states; ++v) {
                    if (finish_area.IsInside(states_.emplace_back(
                            geom::Pose(
                                latitude_pose.pos,
                                latitude_pose.dir.angle() + geom::Angle(params_.forward_yaw[k])),
                            params_.velocity[v]))) {
                        finish_states_.insert(&states_.back());
                    }
                }
            }
            for (size_t k = 0; k < params_.backward_yaw.total_states; ++k) {
                for (size_t v = 0; v < params_.velocity.total_states; ++v) {
                    if (finish_area.IsInside(states_.emplace_back(
                            geom::Pose(
                                latitude_pose.pos,
                                latitude_pose.dir.angle() + geom::Angle(params_.backward_yaw[k])),
                            params_.velocity[v]))) {
                        finish_states_.insert(&states_.back());
                    }
                }
            }
        }
    }

    return *this;
}

const StateSpace::Params& StateSpace::GetParams() const noexcept { return params_; }

const State& StateSpace::GetStartState() const noexcept { return states_.front(); }

const std::unordered_set<const State*>& StateSpace::GetFinishStates() const noexcept {
    return finish_states_;
}

const States& StateSpace::GetStates() const noexcept { return states_; }

void StateSpace::Clear() noexcept {
    states_.clear();
    finish_states_.clear();
}

};  // namespace truck::trajectory_planner