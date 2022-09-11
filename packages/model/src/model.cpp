#include "model/model.h"

#include <cmath>

namespace truck::model {

Model::Model(const std::string& config_path) : params_(config_path) {
    cache_.width_half = params_.wheel_base.width / 2;

    {
        const double tan_inner = tan(abs(params_.limits.steering.inner));
        const double tan_outer = tan(abs(params_.limits.steering.outer));

        const double max_abs_rear_curvature = std::min(
            tan_inner / (params_.wheel_base.length - cache_.width_half * tan_inner),
            tan_outer / (params_.wheel_base.length + cache_.width_half * tan_outer));

        auto rearToBaseCurvature = [&](double C) {
            return C / std::sqrt(1 + squared(C * cache_.width_half));
        };

        cache_.max_abs_curvature =
            std::min(rearToBaseCurvature(max_abs_rear_curvature), params_.limits.max_abs_curvature);
    }
}

Twist Model::baseToRearTwist(Twist twist) const {
    const double ratio = std::sqrt(1 - squared(twist.curvature * params_.wheel_base.base_to_rear));
    return {twist.curvature / ratio, twist.velocity * ratio};
}

Limits<double> Model::baseVelocityLimits() const { return params_.limits.velocity; }

double Model::baseMaxAbsCurvature() const { return cache_.max_abs_curvature; }

Limits<geom::Angle> Model::leftSteeringLimits() const {
    return {-params_.limits.steering.inner, params_.limits.steering.outer};
}

Limits<geom::Angle> Model::rightSteeringLimits() const {
    return {-params_.limits.steering.outer, params_.limits.steering.inner};
}

Steering Model::rearTwistToSteering(Twist twist) const {
    const double first = twist.curvature * params_.wheel_base.length;
    const double second = twist.curvature * cache_.width_half;

    return Steering {
        geom::Angle::fromRadians(std::atan2(first, 1 - second)),
        geom::Angle::fromRadians(std::atan2(first, 1 + second))};
}

WheelVelocity Model::rearTwistToWheelVelocity(Twist twist) const {
    const double ratio = twist.curvature * cache_.width_half;

    return WheelVelocity {
        geom::Angle{(1 - ratio) * twist.velocity / params_.wheel_radius},
        geom::Angle{(1 + ratio) * twist.velocity / params_.wheel_radius}};
}

double Model::linearVelocityToMotorRPS(double velocity) const {
    return velocity / params_.wheel_radius / M_PI / params_.gear_ratio;
}

double Model::gearRatio() const { return params_.gear_ratio; }

}  // namespace truck::model
