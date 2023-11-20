#include "model/model.h"

#include <cmath>

namespace truck::model {

namespace {

double getBaseToRearRatio(double base_curvature, double base_to_rear) {
    return std::sqrt(1 - squared(base_curvature * base_to_rear));
}

double getRearToBaseRatio(double rear_curvature, double base_to_rear) {
    return std::sqrt(1 + squared(rear_curvature * base_to_rear));
}

double rearToBaseCurvature(double rear_curvature, double base_to_rear) {
    const double ratio = getRearToBaseRatio(rear_curvature, base_to_rear);
    return rear_curvature / ratio;
}

} // namespace

Model::Model(const std::string& config_path) : params_(config_path) {
    cache_.width_half = params_.wheel_base.width / 2;

    {
        const double tan_inner = tan(abs(params_.limits.steering.inner));
        const double tan_outer = tan(abs(params_.limits.steering.outer));

        const double max_abs_rear_curvature = std::min(
            tan_inner / (params_.wheel_base.length - cache_.width_half * tan_inner),
            tan_outer / (params_.wheel_base.length + cache_.width_half * tan_outer));

        cache_.max_abs_curvature =
            std::min(rearToBaseCurvature(max_abs_rear_curvature, params_.wheel_base.base_to_rear), 
            params_.limits.max_abs_curvature);

        const double steering_limit 
            = std::atan2(max_abs_rear_curvature, params_.wheel_base.length);
        cache_.middle_steering_limits = {-steering_limit, steering_limit};

        cache_.base_curvature_limits = {-cache_.max_abs_curvature, cache_.max_abs_curvature};
    }
}

Twist Model::baseToRearTwist(Twist twist) const {
    const double ratio = getBaseToRearRatio(twist.curvature, params_.wheel_base.base_to_rear);
    return {twist.curvature / ratio, twist.velocity * ratio};
}

Twist Model::rearToBaseTwist(Twist twist) const {
    const double ratio = getRearToBaseRatio(twist.curvature, params_.wheel_base.base_to_rear);
    return {twist.curvature / ratio, twist.velocity * ratio};
}

double Model::baseToRearAcceleration(double acceleration, double base_curvature) const {
    const double ratio = getBaseToRearRatio(base_curvature, params_.wheel_base.base_to_rear);
    return acceleration * ratio;
}

ServoAngles Model::servoHomeAngles() const { return params_.servo_home_angles; }

double Model::baseMaxAbsCurvature() const { return cache_.max_abs_curvature; }

double Model::steeringVelocity() const { return params_.limits.steering_velocity; }

double Model::baseMaxAcceleration() const { return params_.limits.max_acceleration; }

double Model::baseMaxDeceleration() const { return params_.limits.max_deceleration; }

Limits<double> Model::baseVelocityLimits() const { return params_.limits.velocity; }

Limits<double> Model::baseCurvatureLimits() const { return cache_.base_curvature_limits; }

Limits<geom::Angle> Model::leftSteeringLimits() const {
    return {-params_.limits.steering.inner, params_.limits.steering.outer};
}

Limits<geom::Angle> Model::rightSteeringLimits() const {

    return {-params_.limits.steering.outer, params_.limits.steering.inner};
}

Limits<double> Model::middleSteeringLimits() const {
    return cache_.middle_steering_limits;
}

Steering Model::rearCurvatureToSteering(double curvature) const {
    const double first = curvature * params_.wheel_base.length;
    const double second = curvature * cache_.width_half;

    return Steering {
        geom::Angle::fromRadians(std::atan2(first, 1)),
        geom::Angle::fromRadians(std::atan2(first, 1 - second)),
        geom::Angle::fromRadians(std::atan2(first, 1 + second))
    };
}

double Model::middleSteeringToRearCurvature(double steering) const {
    return tan(steering) / params_.wheel_base.length;
}

Steering Model::rearTwistToSteering(Twist twist) const {
    return rearCurvatureToSteering(twist.curvature);
}

WheelVelocity Model::rearTwistToWheelVelocity(Twist twist) const {
    const double ratio = twist.curvature * cache_.width_half;

    return WheelVelocity {
        geom::Angle{(1 - ratio) * twist.velocity / params_.wheel.radius},
        geom::Angle{(1 + ratio) * twist.velocity / params_.wheel.radius}
    };
}

double Model::linearVelocityToMotorRPS(double velocity) const {
    return velocity / params_.wheel.radius / M_PI / params_.gear_ratio / 2;
}

double Model::motorRPStoLinearVelocity(double rps) const {
    return rps * params_.wheel.radius * M_PI * params_.gear_ratio * 2;
}

double Model::gearRatio() const { return params_.gear_ratio; }

const Shape& Model::shape() const { return params_.shape; }

const WheelBase& Model::wheelBase() const { return params_.wheel_base; }

const Wheel& Model::wheel() const { return params_.wheel; }

}  // namespace truck::model
