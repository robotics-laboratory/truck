#pragma once

#include "geom/angle.h"
#include "common/math.h"

#include "yaml-cpp/yaml.h"

#include <boost/assert.hpp>

namespace truck::model {

struct WheelBase {
    WheelBase(const YAML::Node& node);

    double width;
    double length;
    double base_to_rear;
};

struct TruckShape {
    TruckShape(const YAML::Node& node);

    double width;
    double length;
    double base_to_rear;
    int circles_count_approx_shape;
};

struct SteeringLimit {
    geom::Angle inner;
    geom::Angle outer;
};

struct VehicleLimits {
    VehicleLimits(const YAML::Node& node);

    double max_abs_curvature;
    SteeringLimit steering;
    Limits<double> velocity;
    Limits<double> acceleration;
};

struct ServoAngles {
    ServoAngles(const YAML::Node& node);

    geom::Angle left;
    geom::Angle right; 
};

struct Params {
    Params(const YAML::Node& node);
    Params(const std::string& config_path);

    WheelBase wheel_base;
    TruckShape truck_shape;
    model::VehicleLimits limits;
    double wheel_radius;
    double gear_ratio;
    ServoAngles servo_home_angles;
};

}  // namespace truck::model