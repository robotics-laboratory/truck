#include "node.hpp"

#include "planner.hpp"
#include "planning_interfaces/msg/scene.hpp"
#include "planning_interfaces/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "single_slot_queue.hpp"

#include <memory>


namespace planning_node {

using namespace planning_interfaces;

using std::placeholders::_1;


struct PlanningNode : public rclcpp::Node {
    PlanningNode() : Node("PlanningNode") {
        scene_subscription = this->create_subscription<msg::Scene>(
            "scene", 10, std::bind(&PlanningNode::new_scene_callback, this, _1)
        );
        
        scene_queue = std::make_shared<SingleSlotQueue<msg::Scene::SharedPtr>>();
        path_publisher = this->create_publisher<msg::Path>("planned_path", 10);
        
        planner_thread = start_planner(scene_queue, path_publisher);
    }

    void stop() {
        scene_queue->stop();
        planner_thread.join();
    }

private:
    void new_scene_callback(const msg::Scene::SharedPtr message) const {
        RCLCPP_INFO(this->get_logger(), "New scene: created_at=%ld", message->meta.created_at);
        if (!message->meta.debug.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "New scene has %lu debug messages", message->meta.debug.size());
            for (auto debug_message : message->meta.debug) {
                RCLCPP_DEBUG(this->get_logger(), "%s", debug_message.c_str());
            }
        }

        scene_queue->put(message);
    }

    rclcpp::Subscription<msg::Scene>::SharedPtr scene_subscription;
    rclcpp::Publisher<msg::Path>::SharedPtr path_publisher;
    std::shared_ptr<SingleSlotQueue<msg::Scene::SharedPtr>> scene_queue;

    std::thread planner_thread;
};

std::thread start_planning_node(int argc, char** argv) {
    auto start_node = [](int argc, char** argv) {
        rclcpp::init(argc, argv);
        std::shared_ptr<PlanningNode> node = std::make_shared<PlanningNode>();
        rclcpp::on_shutdown([node]() {
            node->stop();
        });
        rclcpp::spin(node);
        rclcpp::shutdown();
    };
    return std::thread{start_node, argc, argv};
}

}
