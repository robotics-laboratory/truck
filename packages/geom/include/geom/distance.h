#pragma once

#include "geom/line.h"
#include "geom/segment.h"
#include "geom/vector.h"
#include "geom/motion_state.h"

namespace truck::geom {

double distanceSq(const Vec2& a, const Vec2& b) noexcept;

double distance(const Vec2& a, const Vec2& b) noexcept;

double distanceSq(const Vec2& p, const Segment& s) noexcept;

double distance(const Vec2& p, const Segment& s) noexcept;

double denormalizedDistance(const Line& l, const Vec2& p) noexcept;

double denormalizedDistance(const Vec2& p, const Line& l) noexcept;

double distance(const Line& l, const Vec2& p) noexcept;

double distance(const Vec2& p, const Line& l) noexcept;

double distanceSq(const MotionState& a, const MotionState& b) noexcept;

double distance(const MotionState& a, const MotionState& b) noexcept;

}  // namespace truck::geom
