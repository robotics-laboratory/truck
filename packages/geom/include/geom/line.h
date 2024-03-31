#pragma once

#include "geom/common.h"
#include "geom/vector.h"

#include <optional>

namespace truck::geom {

// ax + by + c = 0
struct Line {
    Line(double a, double b, double c) : a(a), b(b), c(c) {}

    Line(const Vec2& norm, double c) : a(norm.x), b(norm.y), c(c) {}

    Vec2 normal() const noexcept { return {a, b}; }

    static Line fromTwoPoints(const Vec2& p1, const Vec2& p2) noexcept {
        Vec2 norm{p2.y - p1.y, p1.x - p2.x};
        return Line(norm, -dot(p1, norm));
    }

    static Line fromPointAndNormal(const Vec2& p, const Vec2& norm) noexcept {
        return Line(norm, -dot(p, norm));
    }

    static Line fromPointAndCollinear(const Vec2& p, const Vec2& col) noexcept {
        return fromPointAndNormal(p, Vec2{col.y, -col.x});
    }

    double a, b, c;
};

bool equal(const Line& l1, const Line& l2, double eps) noexcept;

std::ostream& operator<<(std::ostream& out, const Line& l) noexcept;

}  // namespace truck::geom
