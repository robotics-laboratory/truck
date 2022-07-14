#include "model/model.h"

#include <cmath>

namespace truck::model {

namespace {

template <typename S>
inline Limits<S> toLimits(const YAML::Node& node) {
    return {node["min"].as<S>(), node["max"].as<S>()};
}

inline SteeringLimit toSteeringLimits(const YAML::Node& node) {
    return {geom::Angle{node["inner"].as<double>()}, geom::Angle{node["outer"].as<double>()}};
}

}  // namespace

Model::Model(const std::string& config_path) : params_(config_path) {
    cache_.width_half = params_.wheel_base.width / 2;

    {
        const double tan_inner = tan(abs(params_.limits.steering.inner));
        const double tan_outer = tan(abs(params_.limits.steering.outer));

        const double max_abs_rear_curvature = std::min(
            tan_inner / (params_.wheel_base.length - cache_.width_half * tan_inner),
            tan_outer / (params_.wheel_base.length + cache_.width_half * tan_outer));

        cache_.max_abs_curvature =
            std::min(rearToBaseCurvature(max_abs_rear_curvature), params_.limits.max_abs_curvature);
    }
}

double Model::rearToBaseCurvature(double C) const {
    return C / std::sqrt(1 + squared(C * params_.wheel_base.base_to_rear));
}

double Model::baseToRearCurvature(double C) const {
    return C / std::sqrt(1 - squared(C * params_.wheel_base.base_to_rear));
}

Limits<double> Model::baseVelocityLimits() const { return params_.limits.velocity; }

double Model::baseMaxAbsCurvature() const { return cache_.max_abs_curvature; }

Steering Model::rearCurvatureToSteering(double C) const {
    const double first = std::abs(C) * params_.wheel_base.length;
    const double second = C * cache_.width_half;

    return Steering {
        geom::Angle::fromRadians(std::atan2(first, 1 - second)),
        geom::Angle::fromRadians(std::atan2(first, 1 + second))};
}

}  // namespace truck::model