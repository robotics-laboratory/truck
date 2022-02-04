#pragma once
#include "planning_interfaces/msg/path.hpp"
#include "planning_interfaces/msg/scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "single_slot_queue.hpp"

#include <memory>
#include <thread>


namespace planning_node {

using namespace planning_interfaces;

std::thread start_planner(
    std::shared_ptr<SingleSlotQueue<msg::Scene::SharedPtr>> scene_queue,
    rclcpp::Publisher<msg::Path>::SharedPtr path_publisher
);

}
