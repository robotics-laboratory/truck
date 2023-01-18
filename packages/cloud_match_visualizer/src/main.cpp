#include <iostream>
#include <stdexcept>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "cloud_match_visualizer/visualization_spinner.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::VisualizationSpinner>());
    rclcpp::shutdown();

    return 0;
}