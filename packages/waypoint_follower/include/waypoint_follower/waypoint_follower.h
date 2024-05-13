#pragma once

#include "common/math.h"
#include "geom/pose.h"
#include "geom/vector.h"

#include <deque>
#include <optional>

namespace truck::waypoint_follower {

struct Waypoint {
    static constexpr uint32_t kNoSeqId = -1;

    Waypoint(uint32_t seq_id, const geom::Vec2& pos);

    uint32_t seq_id;
    geom::Vec2 pos;
};

struct LinkedPose {
    LinkedPose(uint32_t wp_seq_id, const geom::Pose& pose);

    uint32_t wp_seq_id;
    geom::Pose pose;
};

class WaypointFollower {
  public:
    struct Parameters {
        double resolution{0.05};
        double check_in_distance{0.1};
        double min_distance{0.3};
    };

    WaypointFollower(const Parameters& params);

    void addEgoWaypoint(const geom::Pose& ego_pose);
    bool addWaypoint(const geom::Vec2& waypoint);
    void update(const geom::Pose& ego_pose);
    void reset();

    // Last waypoint may be removed explicitly!
    bool hasWaypoints() const;
    bool isReadyToFinish(const geom::Pose& ego_pose) const;

    const std::deque<LinkedPose>& path() const;
    const std::deque<Waypoint>& waypoints() const;

  private:
    Parameters params_{};

    struct State {
        uint32_t wp_seq_id;
        std::deque<LinkedPose> path;
        std::deque<Waypoint> waypoints;
    } state_;
};

}  // namespace truck::waypoint_follower
