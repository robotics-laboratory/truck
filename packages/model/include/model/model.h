#pragma once

#include "common/math.h"
#include "geom/angle.h"
#include "geom/pose.h"
#include "model/params.h"
#include "yaml-cpp/yaml.h"

#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

namespace truck::model {

struct Steering {
    geom::Angle left;
    geom::Angle right;
};

struct WheelVelocity {
    geom::Angle left;
    geom::Angle right;
};

struct Twist {
  double curvature;
  double velocity;
};

class Model {
  public:
    Model(const std::string& config_path);

    // Limits
    double baseMaxAbsCurvature() const;
    Limits<geom::Angle> leftSteeringLimits() const;
    Limits<geom::Angle> rightSteeringLimits() const;
    Limits<double> baseVelocityLimits() const;
    Limits<double> baseAccelerationLimits() const;
    ServoAngles servoHomeAngles() const;

    double gearRatio() const;

    const Shape& shape() const;
    const WheelBase& wheelBase() const;

    Twist baseToRearTwist(Twist twist) const;
    Steering rearTwistToSteering(Twist twist) const;
    WheelVelocity rearTwistToWheelVelocity(Twist twist) const;
    double linearVelocityToMotorRPS(double velocity) const;

  private:
    struct Cache {
        double width_half;
        double max_abs_curvature;
    } cache_;

    Params params_;
};

}  // namespace truck::model