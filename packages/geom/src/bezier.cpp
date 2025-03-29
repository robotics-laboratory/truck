#include "geom/bezier.h"
#include "geom/distance.h"

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

MotionStates compose_bezier1(const std::vector<Vec2>& pts, size_t n) {
    VERIFY(pts.size() >= 2);

    MotionStates spline;
    spline.reserve(pts.size() * n);

    for (size_t i = 1; i < pts.size(); ++i) {
        auto chunk = bezier1(pts[i - 1], pts[i], n);
        spline.insert(spline.end(), chunk.begin() + (i == 1 ? 0 : 1), chunk.end());
    }

    return spline;
}

MotionStates compose_bezier2(const std::vector<Vec2>& pts, size_t n) {
    VERIFY(pts.size() >= 3);
    VERIFY((pts.size() - 3) % 2 == 0);

    MotionStates spline;
    spline.reserve(pts.size() * n);

    for (size_t i = 2; i < pts.size(); i += 2) {
        auto chunk = bezier2(pts[i - 2], pts[i - 1], pts[i], n);
        spline.insert(spline.end(), chunk.begin() + (i == 2 ? 0 : 1), chunk.end());
    }

    return spline;
}

MotionStates compose_bezier3(const std::vector<Vec2>& pts, size_t n) {
    VERIFY(pts.size() >= 4);
    VERIFY((pts.size() - 4) % 3 == 0);

    MotionStates spline;
    spline.reserve(pts.size() * n);

    for (size_t i = 3; i < pts.size(); i += 3) {
        auto chunk = bezier3(pts[i - 3], pts[i - 2], pts[i - 1], pts[i], n);
        spline.insert(spline.end(), chunk.begin() + (i == 3 ? 0 : 1), chunk.end());
    }

    return spline;
}

MotionStates catmul_rom(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n) {
    MotionStates points;

    double alpha = 0.5;  // Adjustable alpha value

    auto get_t = [&](double t, double alpha, const Vec2& p0, const Vec2& p1) -> double {
        return t + std::pow(distanceSq(p0, p1), alpha * 0.5);
    };

    auto eval = [&](const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double t

                    ) -> MotionState {
        double t0 = 0.0;
        double t1 = get_t(t0, alpha, p0, p1);
        double t2 = get_t(t1, alpha, p1, p2);
        double t3 = get_t(t2, alpha, p2, p3);

        t = interpolate(t1, t2, t);

        Vec2 A1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0));
        Vec2 A2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1));
        Vec2 A3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2));
        Vec2 B1 = A1 * ((t2 - t) / (t2 - t0)) + A2 * ((t - t0) / (t2 - t0));
        Vec2 B2 = A2 * ((t3 - t) / (t3 - t1)) + A3 * ((t - t1) / (t3 - t1));

        Vec2 position = B1 * ((t2 - t) / (t2 - t1)) + B2 * ((t - t1) / (t2 - t1));

        Vec2 derivative = (B2 - B1) * (1 / (t2 - t1));

        Vec2 second_derivative =
            (((A3 - A2) * ((t - t1) / (t3 - t1)) + derivative * ((t3 - t) / (t3 - t1)))
             - derivative * ((t2 - t) / (t2 - t1)))
            * (1 / (t2 - t1));

        double curvature = cross(derivative, second_derivative) / pow(derivative.len(), 3);

        return MotionState{
            .pos = position,
            .dir = AngleVec2::fromVector(derivative),
            .curvature = curvature,
        };
    };

    for (size_t i = 0; i < n; ++i) {
        double t = static_cast<double>(i) / (n - 1);
        points.push_back(eval(p0, p1, p2, p3, t));
    }

    return points;
}

MotionStates compose_catmul_rom(const std::vector<Vec2>& pts, size_t n) {
    VERIFY(pts.size() >= 2);

    const std::size_t k = pts.size() - 1;
    MotionStates spline;
    spline.reserve(n * k);

    for (size_t i = 1; i < pts.size(); ++i) {
        Vec2 p0 = i > 1 ? pts[i - 2] : pts[0] - (pts[1] - pts[0]);
        Vec2 p1 = pts[i - 1];
        Vec2 p2 = pts[i - 0];
        Vec2 p3 = i < pts.size() - 1 ? pts[i + 1] : pts[k] - (pts[k] - pts[k - 1]);

        auto chunk = catmul_rom(p0, p1, p2, p3, n);
        spline.insert(spline.end(), chunk.begin() + (i == 1 ? 0 : 1), chunk.end());
    }

    return spline;
}
}  // namespace truck::geom
