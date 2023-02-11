#include <iostream>
#include <stdexcept>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "laser_scan_matcher/visualization_spinner.hpp"
#include "laser_scan_matcher/odometry_computer.hpp"

int main(int argc, char* argv[])
{
    bool manual = 0;
    rclcpp::init(argc, argv);

    for(int i = 1; i < argc; i++)
    {
        if(strcmp(argv[i], "--mode") == 0 && argc > i + 1)
        {
            if (strcmp(argv[i+1], "odometry_computer") == 0)
            {
                manual = 1;
                std::cout << "Starting the odometry computer.";
                rclcpp::spin(std::make_shared<cmt::OdometryComputer>());
            }
            else if (strcmp(argv[i+1], "visualization_spinner") == 0)
            {
                manual = 1;
                std::cout << "Starting the visualization spinner.";
                rclcpp::spin(std::make_shared<cmt::VisualizationSpinner>());
            }
            else break;
        }
    }
   
    if(!manual)
    {
        std::cout << "Starting the odometry computer by default.";
        rclcpp::spin(std::make_shared<cmt::OdometryComputer>());
    }
    
    rclcpp::shutdown();

    return 0;
}