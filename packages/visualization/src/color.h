#pragma once

#include "truck_interfaces/msg/control_mode.hpp"

#include <std_msgs/msg/color_rgba.hpp>

namespace truck::visualization::color {

std_msgs::msg::ColorRGBA make(float r, float g, float b, float a = 1.0);

std_msgs::msg::ColorRGBA white(float alpha = 1.0);
std_msgs::msg::ColorRGBA gray(float alpha = 1.0);
std_msgs::msg::ColorRGBA red(float alpha = 1.0);
std_msgs::msg::ColorRGBA green(float alpha = 1.0);
std_msgs::msg::ColorRGBA blue(float alpha = 1.0);

std_msgs::msg::ColorRGBA make(const truck_interfaces::msg::ControlMode& mode);

std_msgs::msg::ColorRGBA plasma(float x, float alpha = 1.0);

} // namespace truck::visualization::color