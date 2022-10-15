#pragma once

#include "truck_interfaces/msg/control.hpp"

namespace truck::pure_pursuit {

const truck_interfaces::msg::Control& STOP() noexcept;

}  // namespace truck::pure_pursuit