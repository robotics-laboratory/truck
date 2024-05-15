#include <waypoint_follower/waypoint_follower.h>

#include "common/math.h"
#include "common/exception.h"
#include "geom/distance.h"
#include "motion/primitive.h"

#include <algorithm>

namespace truck::waypoint_follower {

Waypoint::Waypoint(uint32_t seq_id, const geom::Vec2& pos) : seq_id(seq_id), pos(pos) {}

LinkedPose::LinkedPose(uint32_t wp_seq_id, const geom::Pose& pose) :
    wp_seq_id(wp_seq_id), pose(pose) {}

namespace {

constexpr size_t kNoIdx = -1;

size_t getEgoSegmentIdx(
    const std::deque<LinkedPose>& points, const geom::Pose& ego_pose, double max_distance) {
    double min_distnace_sq = squared(max_distance);
    size_t ego_segment_idx = kNoIdx;

    for (size_t end = 1; end < points.size(); ++end) {
        const size_t begin = end - 1;
        const geom::Segment segment = {points[begin].pose.pos, points[end].pose.pos};
        const double distance_sq = geom::distanceSq(ego_pose.pos, segment);
        // get closest segment
        if (distance_sq >= min_distnace_sq) {
            if (ego_segment_idx == kNoIdx) {
                continue;
            }

            break;
        }

        min_distnace_sq = distance_sq;
        ego_segment_idx = begin;
    }

    return ego_segment_idx;
}

// return end waypoint seq_id
uint32_t cutByEgoPose(std::deque<LinkedPose>& points, const geom::Pose& ego_pose, double distance) {
    if (points.empty()) {
        return Waypoint::kNoSeqId;
    }

    const size_t ego_segment_idx = getEgoSegmentIdx(points, ego_pose, distance);
    if (ego_segment_idx == kNoIdx) {
        return Waypoint::kNoSeqId;
    }

    VERIFY(ego_segment_idx + 1 < points.size());
    auto end = points.begin() + ego_segment_idx;

    const int end_seq_id = end->wp_seq_id;
    points.erase(points.begin(), end);

    return end_seq_id;
}

}  // namespace

WaypointFollower::WaypointFollower(const Parameters& params) : params_(params) { reset(); }

void WaypointFollower::reset() {
    state_.wp_seq_id = 0;
    state_.waypoints.clear();
    state_.path.clear();
}

bool WaypointFollower::isReadyToFinish(const geom::Pose& ego_pose) const {
    return state_.waypoints.size() == 1 && state_.path.size() >= 1
           && geom::distanceSq(ego_pose.pos, state_.path.back().pose.pos)
                  < squared(params_.check_in_distance);
}

void WaypointFollower::addEgoWaypoint(const geom::Pose& ego_pose) {
    VERIFY(state_.waypoints.empty());
    VERIFY(state_.path.empty());

    state_.waypoints.emplace_back(state_.wp_seq_id, ego_pose.pos);
    state_.path.emplace_back(state_.wp_seq_id, ego_pose);
    ++state_.wp_seq_id;
}

bool WaypointFollower::addWaypoint(const geom::Vec2& waypoint) {
    VERIFY(!state_.waypoints.empty());
    VERIFY(!state_.path.empty());

    const auto& pose = state_.path.back().pose;

    const double dist = geom::distance(waypoint, pose.pos);
    if (dist < params_.min_distance) {
        return false;
    }

    const auto poses = geom::findMotion(pose, waypoint, dist / 2, params_.resolution);
    state_.waypoints.emplace_back(state_.wp_seq_id, waypoint);
    for (size_t i = 1; i < poses.size(); ++i) {
        state_.path.emplace_back(state_.wp_seq_id, poses[i]);
    }
    ++state_.wp_seq_id;

    return true;
}

void WaypointFollower::update(const geom::Pose& ego_pose) {
    const uint32_t end_wp_id = cutByEgoPose(state_.path, ego_pose, params_.check_in_distance);

    if (end_wp_id != Waypoint::kNoSeqId) {
        auto& waypoints = state_.waypoints;
        while (!waypoints.empty() && waypoints.front().seq_id < end_wp_id) {
            state_.waypoints.pop_front();
        }
    }
}

const std::deque<LinkedPose>& WaypointFollower::path() const { return state_.path; }

const std::deque<Waypoint>& WaypointFollower::waypoints() const { return state_.waypoints; }

bool WaypointFollower::hasWaypoints() const { return !state_.waypoints.empty(); }

}  // namespace truck::waypoint_follower
