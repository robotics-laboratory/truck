#include "perf/node.h"
#include "perf/stat.h"

#include "truck_msgs/msg/cpu_stat.hpp"
#include "truck_msgs/msg/mem_stat.hpp"

#include <chrono>

namespace truck::perf {

PerfStatNode::PerfStatNode() : rclcpp::Node("PerfStat") {
    constexpr auto period = std::chrono::duration<double>(0.2);
    stat_signal_ = this->create_publisher<truck_msgs::msg::PerfStat>("/perf/stat", 10);
    timer_ = this->create_wall_timer(period, std::bind(&PerfStatNode::OnTimer, this));
    RCLCPP_INFO(this->get_logger(), "PerfStatNode started (%.2f Hz)", 1.0 / period.count());
}

namespace {

truck_msgs::msg::MemStat toMsg(const MemStat& stat) {
    auto msg = truck_msgs::msg::MemStat();
    msg.total = stat.total;
    msg.free = stat.free;
    msg.usage = stat.usageRatio();
    return msg;
}

truck_msgs::msg::CpuStat toMsg(const CpuStat& stat) {
    auto msg = truck_msgs::msg::CpuStat();
    msg.id = stat.id;
    msg.usage = stat.usageRatio();
    return msg;
}

}  // namespace

void PerfStatNode::OnTimer() {
    auto stat = Stat::read();

    if (stat_) {
        auto msg = truck_msgs::msg::PerfStat();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base";

        for (size_t i = 0; i < stat.cpu.size(); ++i) {
            msg.cpu.push_back(toMsg(stat.cpu[i] - stat_->cpu[i]));
        }

        msg.mem = toMsg(stat.mem);
        stat_signal_->publish(msg);
    }

    stat_ = std::move(stat);
}

}  // namespace truck::perf
