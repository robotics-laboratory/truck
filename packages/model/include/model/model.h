#pragma once

#include "common/math.h"
#include "geom/angle.h"
#include "model/params.h"

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
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
    Limits<double> baseAccelerationLimits() const;
    ServoAngles servoHomeAngles() const;

    double gearRatio() const;

    const Shape& shape() const;
    const WheelBase& wheelBase() const;

    Twist baseToRearTwist(Twist twist) const;
    Steering rearTwistToSteering(Twist twist) const;
    WheelVelocity rearTwistToWheelVelocity(Twist twist) const;
    double linearVelocityToMotorRPS(double velocity) const;
    double motorRPStoLinearVelocity(double rps) const;

  private:
    struct Cache {
        double width_half;
        double max_abs_curvature;
    } cache_;

    Params params_;
};

inline auto load(rclcpp::Logger logger, const std::string& path) {
    RCLCPP_INFO(logger, "load model: %s", path.c_str());
    return Model(path);
}

inline auto makeUniquePtr(rclcpp::Logger logger, const std::string& path) {
    RCLCPP_INFO(logger, "load model: %s", path.c_str());
    return std::make_unique<Model>(path);
}

}  // namespace truck::model
