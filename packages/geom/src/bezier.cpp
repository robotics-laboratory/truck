#include "geom/bezier.h"

#include "common/exception.h"
#include "common/math.h"

namespace truck::geom {

Poses CurvePoses::AsPoses() const noexcept {
    Poses poses;
    poses.reserve(this->size());

    for (const auto& curve_pose : *this) {
        poses.emplace_back(curve_pose.pose);
    }

    return poses;
}

CurvePoses bezier1(const Vec2& p0, const Vec2& p1, size_t n) {
    VERIFY(n > 2);

    const auto dir = AngleVec2::fromVector(p1 - p0);

    CurvePoses poses;
    poses.reserve(n);

    poses.emplace_back(Pose{p0, dir}, 0.0);

    for (size_t i = 1; i < n - 1; ++i) {
        const double t = double(i) / (n - 1);
        const double t_1 = 1 - t;
        poses.emplace_back(Pose{p0 * t_1 + p1 * t, dir}, 0.0);
    }

    poses.emplace_back(Pose{p1, dir}, 0.0);

    return poses;
}

CurvePoses bezier1(const Vec2& p0, const Vec2& p1, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezier1(p0, p1, n);
}

CurvePoses bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, size_t n) {
    VERIFY(n > 2);

    CurvePoses poses;
    poses.reserve(n);

    poses.emplace_back(Pose{p0, AngleVec2::fromVector(p1 - p0)}, 0.0);

    for (size_t i = 1; i < n - 1; ++i) {
        const double t = double(i) / (n - 1);
        const double t_1 = 1 - t;

        const auto pos = p0 * t_1 * t_1 + p1 * 2 * t * t_1 + p2 * t * t;
        const auto derivative = 2 * t_1 * (p1 - p0) + 2 * t * (p2 - p1);
        const auto derivative_2 = 2 * (p2 - 2 * p1 + p0);

        const auto curvature = cross(derivative, derivative_2) / std::pow(derivative.len(), 3);

        poses.emplace_back(Pose{pos, AngleVec2::fromVector(derivative)}, curvature);
    }

    poses.emplace_back(Pose{p2, AngleVec2::fromVector(p2 - p1)}, 0.0);

    return poses;
}

CurvePoses bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len() + (p2 - p1).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezier2(p0, p1, p2, n);
}

CurvePoses bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n) {
    VERIFY(n > 2);

    CurvePoses poses;
    poses.reserve(n);

    poses.emplace_back(Pose{p0, AngleVec2::fromVector(p1 - p0)}, 0.0);

    for (size_t i = 1; i < n - 1; ++i) {
        const double t = double(i) / (n - 1);
        const double t2 = t * t;

        const double t_1 = 1 - t;
        const double t_2 = t_1 * t_1;

        const auto pos = p0 * t_2 * t_1 + p1 * 3 * t * t_2 + p2 * 3 * t2 * t_1 + p3 * t2 * t;
        const auto derivative = 3 * t_2 * (p1 - p0) + 6 * t_1 * t * (p2 - p1) + 3 * t2 * (p3 - p2);
        const auto derivative_2 = 6 * t_1 * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1);

        const auto curvature = cross(derivative, derivative_2) / std::pow(derivative.len(), 3);

        poses.emplace_back(Pose{pos, AngleVec2::fromVector(derivative)}, curvature);
    };

    poses.emplace_back(Pose{p3, AngleVec2::fromVector(p3 - p2)}, 0.0);

    return poses;
}

CurvePoses bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len() + (p2 - p1).len() + (p3 - p2).len();
    const size_t n = 1 + std::max(ceil<size_t>(dist / step), static_cast<size_t>(2));
    return bezier3(p0, p1, p2, p3, n);
}

}  // namespace truck::geom
