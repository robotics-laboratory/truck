#include "geom/angle.h"
#include "geom/pose.h"

#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

namespace truck::simulator {

using namespace std::chrono_literals;

class TruckMarker {
    public:
        TruckMarker(
            const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher,
            const double length, const double width, const double height, 
            const float red, const float green, const float blue);
        void publish(const geom::Pose pose, const geom::Angle steering, const rclcpp::Time time);

    private:
        void createBody(const double height, const float red, const float green, const float blue);
        void createWheels(const double height, const float red, const float green, const float blue);
        void updateWheelsPosition(const double x, const double y, 
            const double orientation_x, const double orientation_y, const rclcpp::Time time);
        void updateBodyPosition(const double x, const double y, 
            const double orientation_x, const double orientation_y, const rclcpp::Time time);
        
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
        visualization_msgs::msg::Marker body_;
        visualization_msgs::msg::Marker wheels_[4];
        double body_length_;
        double body_width_;
};

} // namespace truck::simulator