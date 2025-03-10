#include "geom/motion_state.h"

namespace truck::geom {
Poses toPoses(const MotionStates& states) {
    Poses poses(states.size());

    std::transform(states.begin(), states.end(), poses.begin(), [](const MotionState& state) {
        return state.pose();
    });

    return poses;
}

Polyline toPolyline(const MotionStates& states) {
    Polyline points(states.size());

    std::transform(states.begin(), states.end(), points.begin(), [](const MotionState& state) {
        return state.pose().pos;
    });

    return points;
}

}  // namespace truck::geom
