#pragma once

#include "common/math.h"
#include "model/params.h"
#include "yaml-cpp/yaml.h"

#include <utility>

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

    geom::Angle steeringVelocity() const;

    // Achtung! All results are clamped with limits!

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