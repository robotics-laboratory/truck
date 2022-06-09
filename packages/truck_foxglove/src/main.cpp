#include <truck_foxglove/odom_translator.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::OdometryTranslator>());
    rclcpp::shutdown();
    return 0;
}
