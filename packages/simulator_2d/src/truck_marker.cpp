#include "simulator_2d/truck_marker.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace truck::simulator {

void TruckMarker::create(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher,
    const double length, const double width, const double height, const float red,
    const float green, const float blue) {

    body_length_ = length;
    body_width_ = width;
    publisher_ = publisher;
    createBody(height, red, green, blue);
    createWheels(height, red, green, blue);
}

void TruckMarker::createBody(
    const double height, const float red, const float green, const float blue) {

    body_.header.frame_id = "odom_ekf";
    body_.id = 0;

    body_.type = visualization_msgs::msg::Marker::CUBE;
    body_.action = visualization_msgs::msg::Marker::ADD;
    body_.lifetime = rclcpp::Duration::from_seconds(0);

    body_.pose.position.x = 0.0;
    body_.pose.position.y = 0.0;
    body_.pose.position.z = 0.0;

    body_.scale.x = body_length_;
    body_.scale.y = body_width_;
    body_.scale.z = height;

    body_.color.a = 1.0;
    body_.color.r = red;
    body_.color.g = green;
    body_.color.b = blue;
}

void setVerticalRotation(geometry_msgs::msg::Quaternion &original_orientation) {
    tf2::Quaternion q_original, q_rotation, q_new;
    tf2::convert(original_orientation , q_original);
    q_rotation.setRPY(M_PI / 2, 0.0, 0.0);
    q_new = q_rotation * q_original;    
    q_new.normalize();
    tf2::convert(q_new, original_orientation);
}

void TruckMarker::createWheels(
    const double height, const float red, const float green, const float blue) {
    
    const double wheel_length = body_length_ / 5;
    const double wheel_width = body_width_ / 4;

    for (int i = 0; i < 4; ++i) {
        wheels_[i].header.frame_id = "odom_ekf";
        wheels_[i].id = 1 + i;

        wheels_[i].type = visualization_msgs::msg::Marker::CYLINDER;
        wheels_[i].action = visualization_msgs::msg::Marker::ADD;
        wheels_[i].lifetime = rclcpp::Duration::from_seconds(0);

        wheels_[i].scale.x = wheel_length;
        wheels_[i].scale.y = height;
        wheels_[i].scale.z = wheel_width;

        wheels_[i].color.a = 1.0;
        wheels_[i].color.r = red;
        wheels_[i].color.g = green;
        wheels_[i].color.b = blue;

        wheels_[i].pose.position.z = 0.0;

        setVerticalRotation(wheels_[i].pose.orientation);
    }
}

void TruckMarker::updateBodyPosition(const double x, const double y, 
    const double orientation_x, const double orientation_y, const rclcpp::Time time) {
    
    body_.header.stamp = time;
    body_.pose.position.x = x;
    body_.pose.position.y = y;
    body_.pose.orientation.x = orientation_x;
    body_.pose.orientation.y = orientation_y;
}

void TruckMarker::updateWheelsPosition(const double x, const double y, 
    const double orientation_x, const double orientation_y, const rclcpp::Time time) {
    
    for (int i = 0; i < 4; ++i) {
        wheels_[i].header.stamp = time;
        wheels_[i].pose.position.x = x;
        wheels_[i].pose.position.y = y;
        wheels_[i].pose.orientation.x = orientation_x;
        wheels_[i].pose.orientation.y = orientation_y;
    }

    const double half_length = body_length_ / 2;
    const double half_width = body_width_ / 2;

    // Front right wheel.
    wheels_[0].pose.position.x += half_length;
    wheels_[0].pose.position.y += half_width;

    // Front left wheel
    wheels_[1].pose.position.x += half_length;
    wheels_[1].pose.position.y += -half_width;

    // Rear right wheel.
    wheels_[2].pose.position.x += -half_length;
    wheels_[2].pose.position.y += half_width;

    // Rear left wheel.
    wheels_[3].pose.position.x += -half_length;
    wheels_[3].pose.position.y += -half_width;
}

void TruckMarker::publish(
    const geom::Pose pose, const geom::Angle steering, const rclcpp::Time time) {

    updateBodyPosition(pose.pos.x, pose.pos.y, steering.radians(), 0, time);
    updateWheelsPosition(pose.pos.x, pose.pos.y, steering.radians(), 0, time);
    publisher_->publish(body_);
    for (int i = 0; i < 4; ++i) {
        publisher_->publish(wheels_[i]);
    }
}

}  // namespace truck::simulator