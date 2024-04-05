#pragma once

#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>


namespace truck::icp_odometry {
	class IcpVisualizationNode : public rclcpp::Node {
	public:
		IcpVisualizationNode() : Node("playback_node") {};

	private:
		bool checkValueRange(float_t value);

		visualization_msgs::msg::Marker dataPointsToMarker(DataPoints dataPoints, std::string frame_id, std_msgs::msg::ColorRGBA color, builtin_interfaces::msg::Time stamp);

		geometry_msgs::msg::Point eigenVectorToPointMsg(Eigen::Vector3f vector);

		visualization_msgs::msg::Marker matchesToSegmentMarker(Matcher::Matches matches, std::string frame_id, DataPoints ref,
															   DataPoints data, Matcher::OutlierWeights outlierWeights, builtin_interfaces::msg::Time stamp);

		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_icp_transformation_reference;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_icp_transformation_data_before;
		std::vector <rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> publishers_icp_transformation_data_before;
		std::vector <rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_icp_transformation_data_before_errors;
		std::vector <rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> publishers_icp_transformation_data_before_segments;
	};
}
