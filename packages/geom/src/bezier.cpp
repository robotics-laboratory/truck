#include "geom/bezier.h"

#include "common/exception.h"
#include "common/math.h"

namespace truck::geom {

Poses bezier1(const Vec2& p0, const Vec2& p1, size_t n) {
    VERIFY(n > 2);

    const auto dir = AngleVec2::fromVector(p1 - p0);

    Poses poses;
    poses.reserve(n);

    poses.emplace_back(p0, dir);

    for (size_t i = 1; i < n - 1; ++i) {
        const double t = static_cast<double>(i) / (n - 1);
        const double t_1 = 1 - t;
        poses.emplace_back(p0 * t_1 + p1 * t, dir);
    }

    poses.emplace_back(p1, dir);

    return poses;
}

Poses bezier1(const Vec2& p0, const Vec2& p1, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezier1(p0, p1, n);
}

Poses bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, size_t n) {
    VERIFY(n > 2);

    Poses poses;
    poses.reserve(n);

    poses.emplace_back(p0, AngleVec2::fromVector(p1 - p0));

    for (size_t i = 1; i < n - 1; ++i) {
        const double t = static_cast<double>(i) / (n - 1);
        const double t_1 = 1 - t;

        const auto pos = p0 * t_1 * t_1 + p1 * 2 * t * t_1 + p2 * t * t;
        const auto derivative = 2 * t_1 * (p1 - p0) + 2 * t * (p2 - p1);

        poses.emplace_back(pos, AngleVec2::fromVector(derivative));
    }

    poses.emplace_back(p2, AngleVec2::fromVector(p2 - p1));

    return poses;
}

Poses bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len() + (p2 - p1).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezier2(p0, p1, p2, n);
}

Poses bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n) {
    VERIFY(n > 2);

    Poses poses;
    poses.reserve(n);

    poses.emplace_back(p0, AngleVec2::fromVector(p1 - p0));

    for (size_t i = 1; i < n - 1; ++i) {
        const double t = static_cast<double>(i) / (n - 1);
        const double t2 = t * t;

        const double t_1 = 1 - t;
        const double t_2 = t_1 * t_1;

        const auto pos = p0 * t_2 * t_1 + p1 * 3 * t * t_2 + p2 * 3 * t2 * t_1 + p3 * t2 * t;
        const auto derivative = 3 * t_2 * (p1 - p0) + 6 * t_1 * t * (p2 - p1) + 3 * t2 * (p3 - p2);

        poses.emplace_back(pos, AngleVec2::fromVector(derivative));
    };

    poses.emplace_back(p3, AngleVec2::fromVector(p3 - p2));

    return poses;
}

Poses bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len() + (p2 - p1).len() + (p3 - p2).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezier3(p0, p1, p2, p3, n);
}

}  // namespace truck::geom
