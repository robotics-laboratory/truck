#pragma once

#include "icp_odometry/common.h"

#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>


namespace truck::icp_odometry {
    class IcpVisualizationNode : public rclcpp::Node {
    public:
        IcpVisualizationNode();

    private:
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_icp_transformation_reference;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_icp_transformation_data_before;
        std::vector <rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> publishers_icp_transformation_data_before;
        std::vector <rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_icp_transformation_data_before_errors;
        std::vector <rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> publishers_icp_transformation_data_before_segments;
    };

    void saveDataPointsToMcap(const DataPoints &, const std::string &, int, char **);

    void saveICPTransformationToMcap(const DataPoints &, const DataPoints &, const DataPoints &, const std::string &);
}
