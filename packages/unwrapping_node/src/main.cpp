#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planning_interfaces/msg/path.hpp"
#include "planning_interfaces/msg/random_seed.hpp"
#include "planning_interfaces/msg/scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"


#include <iostream>
#include <chrono>
#include <memory>
#include <random>


namespace unwrapping_node {

using std::placeholders::_1;

struct UnwrappingNode : public rclcpp::Node {
    UnwrappingNode() : Node("UnwrappingNode") {
        path_subscription = create_subscription<planning_interfaces::msg::Path>(
            "path", 10, std::bind(&UnwrappingNode::new_path_callback, this, _1)
        );
        raw_path_publisher = create_publisher<nav_msgs::msg::Path>("raw_path", 10);

        scene_subscription = create_subscription<planning_interfaces::msg::Scene>(
            "scene", 10, std::bind(&UnwrappingNode::new_scene_callback, this, _1)
        );
        raw_occupancy_grid_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("raw_occupancy_grid", 10);

        random_scene_subscriber = create_subscription<planning_interfaces::msg::RandomSeed>(
            "random_scene", 10, std::bind(&UnwrappingNode::new_random_scene_callback, this, _1)
        );
        scene_publisher = create_publisher<planning_interfaces::msg::Scene>("scene", 10);
    }

private:
    void new_path_callback(const planning_interfaces::msg::Path::SharedPtr message) const {
        RCLCPP_INFO(get_logger(), "New path: created_at=%ld", message->created_at);
        raw_path_publisher->publish(message->path);
    }

    void new_scene_callback(const planning_interfaces::msg::Scene::SharedPtr message) const {
        RCLCPP_INFO(get_logger(), "New scene: created_at=%ld", message->created_at);
        raw_occupancy_grid_publisher->publish(message->occupancy_grid);
    }

    // temporary random scene generator for debug
    void new_random_scene_callback(const planning_interfaces::msg::RandomSeed::SharedPtr message) const {
        RCLCPP_INFO(get_logger(), "Generating random scene: seed=%ld, probability=%.3f", message->seed, message->probability);
        planning_interfaces::msg::Scene scene;

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

    rclcpp::Subscription<planning_interfaces::msg::Path>::SharedPtr path_subscription;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_publisher;
    
    rclcpp::Subscription<planning_interfaces::msg::Scene>::SharedPtr scene_subscription;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_occupancy_grid_publisher;
    
    rclcpp::Subscription<planning_interfaces::msg::RandomSeed>::SharedPtr random_scene_subscriber;
    rclcpp::Publisher<planning_interfaces::msg::Scene>::SharedPtr scene_publisher;
};

}

int main(int argc, char** argv) {
    std::cout << "Starting unwrapping node" << std::endl;

    rclcpp::init(argc, argv);
    std::shared_ptr<unwrapping_node::UnwrappingNode> node = std::make_shared<unwrapping_node::UnwrappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
