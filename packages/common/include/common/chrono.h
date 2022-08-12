#pragma once

#include <rclcpp/time.hpp>

#include <chrono>

namespace truck {

template<class Rep, class Period>
rclcpp::Duration toRosDuration(std::chrono::duration<Rep, Period> time) {
    return rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(time));
}

inline std::chrono::nanoseconds nanoseconds(rclcpp::Duration duration) {
    return std::chrono::nanoseconds(duration.nanoseconds());
}

std::chrono::milliseconds milliseconds(rclcpp::Duration duration) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(nanoseconds(duration));
}

} // namespace truck