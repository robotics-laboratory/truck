#include "pure_pursuit/curvature_planner.hpp"

using geom::Vec2d;

namespace pure_pursuit {

std::optional<CurvaturePlanner::Plan> CurvaturePlanner::calculatePlan(Vec2d start, Vec2d target, double start_yaw, double start_sigma, double linear_velocity, double angular_velocity) {
    auto true_curvature = std::tan(start_sigma) / length;
    auto initial_required_arc = geom::Arc::fromTwoPointsAndTangentalVector(start, target, Vec2d(start_yaw));
    double max_sigma;
    if (!initial_required_arc) {
        max_sigma = std::copysign(max_wheels_angle, true_curvature);
    } else {
        double required_curvature = initial_required_arc->getSignedCurvature();
        if (geom::near(true_curvature, required_curvature)) {
            return Plan{*initial_required_arc, 0};
        }
        if (std::signbit(required_curvature) != std::signbit(true_curvature)) {
            max_sigma = std::copysign(max_wheels_angle, required_curvature);
        } else {
            if (std::abs(required_curvature) > std::abs(true_curvature)) {
                max_sigma = std::copysign(max_wheels_angle, required_curvature);
            } else {
                max_sigma = 0;
            }
        }
    }
    double l = start_sigma, r = max_sigma;
    if (std::signbit(l) != std::signbit(r)) {
        l = 0;
    }
    if (l > r) {
        std::swap(l, r);
        angular_velocity = -angular_velocity;
    }
    std::optional<Plan> result;
    for (int i = 0; i < 20; ++i) {
        double m = (l + r) / 2;
        double dx = integrator.getAnaliticFuncIntegral(cos_coefs, start_sigma, m, linear_velocity / angular_velocity) * linear_velocity;
        double dy = integrator.getAnaliticFuncIntegral(sin_coefs, start_sigma, m, linear_velocity / angular_velocity) * linear_velocity;
        double dyaw = theta(m) - theta(start_sigma);
        Vec2d position = start + Vec2d(dx, dy).rotate(start_yaw);
        auto arc = geom::Arc::fromTwoPointsAndTangentalVector(position, target, Vec2d(start_yaw + dyaw));
        if (!arc) {
            r = m;
            continue;
        }
        result = Plan{*arc, m - start_sigma};
        double required_curvature = initial_required_arc->getSignedCurvature();
        double true_curvature = std::tan(m) / length;
        if (std::signbit(angular_velocity) == std::signbit(required_curvature - true_curvature)) {
            l = m;
        } else {
            r = m;
        }
    }
    return result;
}

}
