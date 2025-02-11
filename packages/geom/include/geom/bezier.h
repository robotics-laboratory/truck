#pragma once

#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::geom {

Poses bezier1(const Vec2& p0, const Vec2& p1, size_t n);
Poses bezier1(const Vec2& p0, const Vec2& p1, double step);

Poses bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, size_t n);
Poses bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, double step);

Poses bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n);
Poses bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double step);

}  // namespace truck::geom
