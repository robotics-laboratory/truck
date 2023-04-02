#include "geom/bezue.h"

#include "common/exception.h"
#include "common/math.h"

namespace truck::geom {

Poses bezue1(const Vec2& p0, const Vec2& p1, size_t n) {
    VERIFY(n > 2);

    const Vec2 dir = (p1 - p0).unit();

    Poses poses;
    poses.reserve(n);

    poses.emplace_back(p0, dir);

    for (size_t i = 1; i < n - 1; ++i) {
        const double t = double(i) / (n - 1);
        const double t_1 = 1 - t;
        poses.emplace_back(p0 * t_1 + p1 * t, dir);
    }

    poses.emplace_back(p1, dir);

    return poses;
}

Poses bezue1(const Vec2& p0, const Vec2& p1, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezue1(p0, p1, n);
}

Poses bezue2(const Vec2& p0, const Vec2& p1, const Vec2& p2, size_t n) {
    VERIFY(n > 2);

    const Vec2 dir10 = (p1 - p0).unit();
    const Vec2 dir21 = (p2 - p1).unit();

    Poses poses;
    poses.reserve(n);

    poses.emplace_back(p0, dir10);

    for (size_t i = 1; i < n - 1; ++i) {
        const double t = double(i) / (n - 1);
        const double t_1 = 1 - t;

        const auto pos = p0 * t_1 * t_1 + p1 * 2 * t * t_1 + p2 * t * t;
        const auto dir = dir10 * t_1 + dir21 * t;

        poses.emplace_back(pos, dir);
    }

    poses.emplace_back(p2, dir21);

    return poses;
}

Poses bezue2(const Vec2& p0, const Vec2& p1, const Vec2& p2, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len() + (p2 - p1).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezue2(p0, p1, p2, n);
}

Poses bezue3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n) {
    VERIFY(n > 2);

    const Vec2 dir10 = (p1 - p0).unit();
    const Vec2 dir21 = (p2 - p1).unit();
    const Vec2 dir32 = (p3 - p2).unit();

    Poses poses;
    poses.reserve(n);

    poses.emplace_back(p0, dir10);

    for (size_t i = 1; i < n - 1; ++i) {
        const double t = double(i) / (n - 1);
        const double t2 = t * t;

        const double t_1 = 1 - t;
        const double t_2 = t_1 * t_1;

        const auto pos = p0 * t_2 * t_1 + p1 * 3 * t * t_2 + p2 * 3 * t2 * t_1 + p3 * t2 * t;
        const auto dir = dir10 * t_2 + dir21 * 2 * t * t_1 + dir32 * t2;

        poses.emplace_back(pos, dir);
    };

    poses.emplace_back(p3, dir32);

    return poses;
}

Poses bezue3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len() + (p2 - p1).len() + (p3 - p2).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezue3(p0, p1, p2, p3, n);
}

}  // namespace truck::geom