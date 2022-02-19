#include "node.hpp"

#include "planner.hpp"
#include "planning_interfaces/msg/scene.hpp"
#include "planning_interfaces/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "single_slot_queue.hpp"

#include <iostream>
#include <memory>


namespace planning_node {

using namespace planning_interfaces;

using std::placeholders::_1;


struct PlanningNode : public rclcpp::Node {
    PlanningNode(std::string config_path) : Node("PlanningNode") {


        scene_subscription = create_subscription<msg::Scene>(
            "scene", 10, std::bind(&PlanningNode::new_scene_callback, this, _1)
        );
        target_subscription = create_subscription<msg::Point>(
            "target", 10, std::bind(&PlanningNode::new_target_callback, this, _1)
        );

        scene_queue = std::make_shared<SingleSlotQueue<msg::Scene::SharedPtr>>();
        target_queue = std::make_shared<SingleSlotQueue<msg::Point::SharedPtr>>();

        path_publisher = create_publisher<msg::Path>("path", 10);
        
        planner_thread = start_planner(scene_queue, target_queue, path_publisher, config_path, get_logger());
    }

    void stop() {
        scene_queue->stop();
        target_queue->stop();
        planner_thread.join();
    }

private:
    void new_scene_callback(const msg::Scene::SharedPtr message) const {
        RCLCPP_INFO(get_logger(), "New scene: created_at=%ld", message->meta.created_at);
        if (!message->meta.debug.empty()) {
            RCLCPP_DEBUG(get_logger(), "New scene has %lu debug messages", message->meta.debug.size());
            for (auto debug_message : message->meta.debug) {
                RCLCPP_DEBUG(get_logger(), "%s", debug_message.c_str());
            }
        }

        scene_queue->put(message);
    }

    void new_target_callback(const msg::Point::SharedPtr message) const {
        RCLCPP_INFO(get_logger(), "New target: x=%.3f, y=%.3f, theta=%.3f", message->x, message->y, message->theta);
        target_queue->put(message);
    }

    rclcpp::Subscription<msg::Scene>::SharedPtr scene_subscription;
    rclcpp::Subscription<msg::Point>::SharedPtr target_subscription;
    std::shared_ptr<SingleSlotQueue<msg::Scene::SharedPtr>> scene_queue;
    std::shared_ptr<SingleSlotQueue<msg::Point::SharedPtr>> target_queue;
    rclcpp::Publisher<msg::Path>::SharedPtr path_publisher;
    
    std::thread planner_thread;
};

std::thread start_planning_node(int argc, char** argv) {
    auto start_node = [](int argc, char** argv) {
        std::string config_path;
        if (argc > 1) {
            config_path = argv[1];

            --argc;
            ++argv;
        }

        std::cout << "Starting planning node" << std::endl;
        
        rclcpp::init(argc, argv);
        std::shared_ptr<PlanningNode> node = std::make_shared<PlanningNode>(config_path);
        rclcpp::on_shutdown([node]() {
            node->stop();
        });
        rclcpp::spin(node);
        rclcpp::shutdown();
    };
    return std::thread{start_node, argc, argv};
}

}
