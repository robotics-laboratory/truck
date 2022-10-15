#include "pure_pursuit/util.h"

namespace truck::pure_pursuit {

const truck_interfaces::msg::Control& STOP() noexcept {
    static const truck_interfaces::msg::Control stop = [] {
        truck_interfaces::msg::Control cmd;

        cmd.velocity = 0;
        cmd.acceleration = 0;
        cmd.curvature = 0;

        return cmd;
    }();

    return stop;
}

}  // namespace truck::pure_pursuit