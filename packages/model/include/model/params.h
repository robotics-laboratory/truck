#pragma once

#include "geom/vector.h"
#include "geom/pose.h"
#include "geom/angle.h"
#include "common/math.h"
#include "model/shape.h"

#include "yaml-cpp/yaml.h"

namespace truck::model {

struct WheelBase {
    WheelBase(const YAML::Node& node);

    double width;
    double length;
    double base_to_rear;
};

struct SteeringLimit {
    geom::Angle inner;
    geom::Angle outer;
};

struct VehicleLimits {
    VehicleLimits(const YAML::Node& node);

    double max_abs_curvature;
    double steering_velocity;
    SteeringLimit steering;
    Limits<double> velocity;
    double max_acceleration;
    double max_deceleration;
};

struct ServoAngles {
    ServoAngles(const YAML::Node& node);

    geom::Angle left;
    geom::Angle right;
};

struct Wheel {
    Wheel(const YAML::Node& node);

    double radius;
    double width;
};

struct Lidar {
    Lidar(const YAML::Node& node);

    geom::Angle angle_min;
    geom::Angle angle_max;
    geom::Angle angle_increment;
    float range_min;
    float range_max;
};

struct Params {
    Params(const YAML::Node& node);
    Params(const std::string& config_path);

    Shape shape;
    WheelBase wheel_base;
    Wheel wheel;
    Lidar lidar;
    VehicleLimits limits;
    double gear_ratio;
    ServoAngles servo_home_angles;
};

}  // namespace truck::model
