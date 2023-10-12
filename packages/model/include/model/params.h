#pragma once

#include "geom/vector.h"
#include "geom/pose.h"
#include "geom/angle.h"
#include "common/math.h"

#include "yaml-cpp/yaml.h"

#include <boost/assert.hpp>

namespace truck::model {

struct Shape {
    Shape();
    Shape(const YAML::Node& node);

    double width;
    double length;
    double base_to_rear;
    int circles_count;

    double radius() const;
    std::vector<geom::Vec2> getCircleDecomposition(const geom::Pose& ego_pose) const;
};

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
    Limits<double> acceleration;
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

struct Params {
    Params(const YAML::Node& node);
    Params(const std::string& config_path);

    Shape shape;
    WheelBase wheel_base;
    VehicleLimits limits;
    Wheel wheel;
    double gear_ratio;
    ServoAngles servo_home_angles;
};

}  // namespace truck::model