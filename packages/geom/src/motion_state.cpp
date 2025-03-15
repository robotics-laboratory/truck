#include "geom/motion_state.h"

namespace truck::geom {
Poses toPoses(const MotionStates& states) {
    Poses poses(states.size());

    std::transform(states.begin(), states.end(), poses.begin(), [](const MotionState& state) {
        return state.pose();
    });

    return poses;
}

}  // namespace truck::geom
