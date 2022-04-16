#include "node.hpp"

#include "planner.hpp"
#include "planning_interfaces/msg/path.hpp"
#include "planning_interfaces/msg/point.hpp"
#include "planning_interfaces/msg/random_seed.hpp"
#include "planning_interfaces/msg/scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shared_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"

#include <iostream>
#include <memory>
#include <random>


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
        random_scene_subscription = create_subscription<msg::RandomSeed>(
            "random_scene", 10, std::bind(&PlanningNode::new_random_scene_callback, this, _1)
        );

        raw_occupancy_grid_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("raw_occupancy_grid", 10);
        scene_publisher = create_publisher<msg::Scene>("scene", 10);

        shared_state = std::make_shared<SharedState>();

        planner_thread = start_planner(this, shared_state, config_path, get_logger());
    }

    void stop() {
        shared_state->stop();
        planner_thread.join();
    }

private:
    void new_scene_callback(const msg::Scene::SharedPtr message) const {
        RCLCPP_INFO(get_logger(), "New scene: created_at=%ld", message->created_at);
        shared_state->set_scene(message);
        raw_occupancy_grid_publisher->publish(message->occupancy_grid);
    }

    void new_target_callback(const msg::Point::SharedPtr message) const {
        RCLCPP_INFO(get_logger(), "New target: x=%.3f, y=%.3f, theta=%.3f", message->x, message->y, message->theta);
        shared_state->set_target(message);
    }

    // temporary random scene generator for debug
    void new_random_scene_callback(const msg::RandomSeed::SharedPtr message) const {
        RCLCPP_INFO(get_logger(), "Generating random scene: seed=%ld, probability=%.3f", message->seed, message->probability);
        msg::Scene scene;

        std::mt19937_64 engine{message->seed};
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        scene.created_at = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        int dx = 20;
        int dy = 20;
        float resolution = 1.0f;
        scene.occupancy_grid.info.resolution = resolution;
        scene.occupancy_grid.info.width = 2 * dx + 1;
        scene.occupancy_grid.info.height = 2 * dy + 1;

        geometry_msgs::msg::Pose origin;
        origin.position.x = -(dx + 0.5) * resolution;
        origin.position.y = -(dy + 0.5) * resolution;
        tf2::Quaternion quart;
        quart.setRPY(0.0, 0.0, 0.0);
        origin.orientation = tf2::toMsg(quart);
        scene.occupancy_grid.info.origin = origin;

        for (int y = -dy; y <= dy; ++y) {
            for (int x = -dx; x <= dx; ++x) {
                if (dist(engine) < message->probability) {
                    scene.occupancy_grid.data.push_back(static_cast<int8_t>(100));
                } else {
                    scene.occupancy_grid.data.push_back(static_cast<int8_t>(0));
                }
            }
        }

        scene_publisher->publish(scene);
    }

    rclcpp::Subscription<msg::Scene>::SharedPtr scene_subscription;
    rclcpp::Subscription<msg::Point>::SharedPtr target_subscription;
    rclcpp::Subscription<msg::RandomSeed>::SharedPtr random_scene_subscription;    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_occupancy_grid_publisher;
    rclcpp::Publisher<planning_interfaces::msg::Scene>::SharedPtr scene_publisher;

    SharedState::SharedPtr shared_state;
    std::thread planner_thread;
};

std::thread start_planning_node(int argc, char** argv) {
    auto start_node = [](int argc, char** argv) {
        // todo: use ros param for config_path
        std::string config_path = "packages/planning_node/config.json";

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
