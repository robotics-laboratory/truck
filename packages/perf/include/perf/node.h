#pragma once

#include "perf/stat.h"

#include "truck_msgs/msg/perf_stat.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>

namespace truck::perf {

class PerfStatNode: public rclcpp::Node {
public:
    PerfStatNode();

    void OnTimer();

private:
    std::optional<Stat> stat_ = {};
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
    rclcpp::Publisher<truck_msgs::msg::PerfStat>::SharedPtr stat_signal_ = nullptr;
};

} // namespace truck::perf
