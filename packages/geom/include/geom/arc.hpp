#pragma once

#include <climits>
#include <cmath>
#include <utility>

#include "geom/common.hpp"
#include "geom/distance.hpp"
#include "geom/vector.hpp"

namespace geom {

class Arc {
private:
    Vec2d start, finish;
    double dist_to_other_side;

    Arc(Vec2d start, Vec2d finish, double dist_to_other_side)
        : start(start), finish(finish), dist_to_other_side(dist_to_other_side) {}

public:
    [[gnu::always_inline, nodiscard, gnu::pure]] static inline Arc fromTwoPointsAndTangentalVector(
        Vec2d start, Vec2d finish, Vec2d tangental, double eps = 1e-9) {
        Vec2d norm = tangental;
        std::swap(norm.x, norm.y);
        norm.x = -norm.x;
        Vec2d delta = finish - start;
        double proj = dot(norm, delta) / delta.len();
        if (near(proj, 0, eps)) {
            return Arc(start, finish, 0);
        }
        Vec2d center = start + norm * delta.len() * proj / 2;
        Vec2d mid = (start + finish) / 2;
        double mid_dist = dist(mid, center);
        double radius = dist(start, center);
        double dist_to_other_side;
        if (dot(tangental, center - mid) > 0)
            dist_to_other_side = radius + mid_dist;
        else
            dist_to_other_side = radius - mid_dist;
        if (cross(finish - start, tangental) < 0) dist_to_other_side = -dist_to_other_side;
        return Arc(start, finish, dist_to_other_side);
    }

    [[gnu::always_inline, nodiscard, gnu::pure]] inline auto getStart() const noexcept {
        return start;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] inline auto getFinish() const noexcept {
        return finish;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] inline double getRadius() const noexcept {
        if (dist_to_other_side == 0) return std::numeric_limits<double>::infinity();
        double l = dist(start, finish) / 2;
        double d = std::abs(dist_to_other_side);
        return (l * l + d * d) / (2 * d);
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] inline double getAngle() const noexcept {
        if (dist_to_other_side == 0) return 0;
        double l = dist(start, finish) / 2;
        return asin(l / getRadius()) * 2;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] inline double getLength() {
        if (dist_to_other_side == 0) return dist(start, finish);
        return getRadius() * getAngle();
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] inline Vec2d getCenter() {
        Vec2d mid = (start + finish) / 2;
        Vec2d offset = (finish - start);
        std::swap(offset.x, offset.y);
        offset.x = -offset.x;
        offset /= offset.len();
        return mid + offset * (dist_to_other_side - getRadius());
    }

    Vec2d getPoint(double ratio) {
        if (dist_to_other_side == 0) {
            return start * (1 - ratio) + finish * ratio;
        }
        Vec2d center = getCenter();
        double start_angle = (start - center).radians();
        double finish_angle = (finish - center).radians();
        double angular_delta = finish_angle - start_angle;
        if (angular_delta < 0 && dist_to_other_side < 0)
            angular_delta += M_PI * 2;
        else if (angular_delta > 0 && dist_to_other_side > 0)
            angular_delta -= M_PI * 2;
        angular_delta *= ratio;
        return center + (start - center).rotate(angular_delta);
    }

    [[gnu::always_inline, nodiscard, gnu::pure]] bool operator==(const Arc& other) const noexcept {
        return start == other.start && finish == other.finish &&
               dist_to_other_side == other.dist_to_other_side;
    }
    [[gnu::always_inline, nodiscard, gnu::pure]] bool operator!=(const Arc& other) const noexcept {
        return !(*this == other);
    }

    [[gnu::always_inline, nodiscard, gnu::pure]] friend inline bool near(const Arc& a, const Arc& b,
                                                                         double eps = 0) noexcept {
        return near(a.start, b.start, eps) && near(a.finish, b.finish, eps) &&
               near(a.dist_to_other_side, b.dist_to_other_side, eps);
    }
};

}  // namespace geom
