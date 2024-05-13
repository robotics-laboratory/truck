#include "color.h"

#include <boost/assert.hpp>

namespace truck::visualization::color {

std_msgs::msg::ColorRGBA make(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

std_msgs::msg::ColorRGBA white(float alpha) {
    return make(1.0, 1.0, 1.0, alpha);
}

std_msgs::msg::ColorRGBA gray(float alpha) {
    return make(0.5, 0.5, 0.5, alpha);
}

std_msgs::msg::ColorRGBA red(float alpha) {
    return make(1.0, 0.0, 0.0, alpha);
}

std_msgs::msg::ColorRGBA green(float alpha) {
    return make(0.0, 1.0, 0.0, alpha);
}

std_msgs::msg::ColorRGBA blue(float alpha) {
    return make(0.0, 0.0, 1.0, alpha);
}

std_msgs::msg::ColorRGBA make(const truck_msgs::msg::ControlMode& mode) {
    switch (mode.mode) {
        case truck_msgs::msg::ControlMode::OFF:
            return red(0.5);
        case truck_msgs::msg::ControlMode::REMOTE:
            return blue(0.5);
        case truck_msgs::msg::ControlMode::AUTO:
            return green(0.5);
        default:
            BOOST_ASSERT_MSG(false, "Unknown mode!");
            return white(0);
    }
}

std_msgs::msg::ColorRGBA plasma(float x, float alpha) {
    std_msgs::msg::ColorRGBA color;

    color.a = alpha;

    if (x < 0.25) {
        color.r = 0;
        color.g = 4 * x;
        color.b = 1;
    } else if (x < 0.5) {
        color.r = 0;
        color.g = 1;
        color.b = 1 + 4 * (0.25 - x);
    } else if (x < 0.75) {
        color.r = 4 * (x - 0.5);
        color.g = 1;
        color.b = 0;
    } else {
        color.r = 1;
        color.g = 1 + 4 * (0.75 - x);
        color.b = 0;
    }

    return color;
}

} // namespace truck::visualization::color
