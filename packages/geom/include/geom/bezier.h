#pragma once

#include "geom/motion_state.h"
#include "geom/vector.h"

namespace truck::geom {

MotionStates bezier1(const Vec2& p0, const Vec2& p1, size_t n);
MotionStates bezier1(const Vec2& p0, const Vec2& p1, double step);

MotionStates bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, size_t n);
MotionStates bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, double step);

MotionStates bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n);
MotionStates bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double step);

MotionStates compose_bezier1(const std::vector<Vec2>& pts, size_t n);
MotionStates compose_bezier2(const std::vector<Vec2>& pts, size_t n);
MotionStates compose_bezier3(const std::vector<Vec2>& pts, size_t n);

MotionStates catmul_rom(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n);
MotionStates compose_catmul_rom(const std::vector<Vec2>& pts, size_t n);

}  // namespace truck::geom
