#include "geom/bezier.h"

#include "common/exception.h"
#include "common/math.h"

namespace truck::geom {

MotionStates bezier1(const Vec2& p0, const Vec2& p1, size_t n) {
    VERIFY(n > 2);

    const auto dir = AngleVec2::fromVector(p1 - p0);

    MotionStates states;
    states.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / (n - 1);
        states.push_back(MotionState{.pos = interpolate(p0, p1, t), .dir = dir, .curvature = 0});
    }

    return states;
}

MotionStates bezier1(const Vec2& p0, const Vec2& p1, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezier1(p0, p1, n);
}

MotionStates bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, size_t n) {
    VERIFY(n > 2);

    MotionStates states;
    states.reserve(n);

    auto eval = [&](const Vec2& p0, const Vec2& p1, const Vec2& p2, double t) -> MotionState {
        const double t_1 = 1 - t;

        const auto position = p0 * t_1 * t_1 + p1 * 2 * t * t_1 + p2 * t * t;
        const auto derivative = 2 * t_1 * (p1 - p0) + 2 * t * (p2 - p1);
        const auto second_derivative = 2 * (p2 - 2 * p1 + p0);
        const double curvature = cross(derivative, second_derivative) / pow(derivative.len(), 3);

        return MotionState{
            .pos = position,
            .dir = AngleVec2::fromVector(derivative),
            .curvature = curvature,
        };
    };

    for (size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / (n - 1);
        states.push_back(eval(p0, p1, p2, t));
    }

    return states;
}

MotionStates bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len() + (p2 - p1).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezier2(p0, p1, p2, n);
}

MotionStates bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n) {
    VERIFY(n > 2);

    std::vector<MotionState> states;
    states.reserve(n);

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

    for (size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / (n - 1);
        states.push_back(eval(p0, p1, p2, p3, t));
    }

    return states;
}

MotionStates bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double step) {
    VERIFY(step > 0);

    const double dist = (p1 - p0).len() + (p2 - p1).len() + (p3 - p2).len();
    const size_t n = 1 + ceil<size_t>(dist / step);
    return bezier3(p0, p1, p2, p3, n);
}

}  // namespace truck::geom
