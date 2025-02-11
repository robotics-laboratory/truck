#pragma once

#include "geom/pose.h"
#include "geom/motion_state.h"
#include <optional>

namespace truck::geom::spline_depr {

std::vector<MotionState> bezier3(
    const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n) {
    VERIFY(n > 2);

    auto eval = [&](const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double t)
        -> MotionState {
        const double t2 = t * t;

        const double t_1 = 1 - t;
        const double t_2 = t_1 * t_1;

        const auto position = p0 * t_2 * t_1 + p1 * 3 * t * t_2 + p2 * 3 * t2 * t_1 + p3 * t2 * t;
        const auto derivative = 3 * t_2 * (p1 - p0) + 6 * t_1 * t * (p2 - p1) + 3 * t2 * (p3 - p2);

        const auto second_derivative =
            6 * (1 - t) * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1);

        const double curvature = cross(derivative, second_derivative) / pow(derivative.len(), 3);

        return MotionState{
            .pos = position,
            .dir = AngleVec2::fromVector(derivative),
            .curvature = curvature,
        };
    };

    std::vector<MotionState> states;
    states.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / (n - 1);
        states.push_back(eval(p0, p1, p2, p3, t));
    }

    return states;
}

// std::optional<std::vector<MotionState>> makeCuvatureConstrainedBezier3(
//     const Pose& from, const Pose& to, double max_curvature, size_t n) {
//     VERIFY(n > 2);

//     auto eval = [&](const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double t)
//         -> MotionState {
//         const double t2 = t * t;

//         const double t_1 = 1 - t;
//         const double t_2 = t_1 * t_1;

//         const auto position = p0 * t_2 * t_1 + p1 * 3 * t * t_2 + p2 * 3 * t2 * t_1 + p3 * t2 * t;
//         const auto derivative = 3 * t_2 * (p1 - p0) + 6 * t_1 * t * (p2 - p1) + 3 * t2 * (p3 - p2);

//         const auto second_derivative =
//             6 * (1 - t) * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1);

//         const double curvature = cross(derivative, second_derivative) / pow(derivative.len(), 3);

//         return MotionState{
//             .position = position,
//             .yaw = AngleVec2::fromVector(derivative),
//             .curvature = curvature,
//         };
//     };

//     auto find_max_curvature = [&](double gamma) {
//         const auto p0 = from.pos;
//         const auto p1 = from.pos + from.dir * gamma;

//         const auto p2 = to.pos;
//         const auto p3 = to.pos + to.dir * gamma;

//         double max_curvature = 0;

//         for (size_t i = 1; i < n - 1; ++i) {
//             const double t = static_cast<double>(i) / (n - 1);
//             auto p = eval(p0, p1, p2, p3, t);
//             max_curvature = std::max(max_curvature, p.curvature);
//         }

//         return max_curvature;
//     };
// }

}  // namespace truck::geom::spline
