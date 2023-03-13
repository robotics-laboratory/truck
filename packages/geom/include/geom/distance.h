#pragma once

#include "geom/line.h"
#include "geom/segment.h"
#include "geom/vector.h"

namespace truck::geom {

double distanceSq(const Vec2& a, const Vec2& b) noexcept;

double distance(const Vec2& a, const Vec2& b) noexcept;

double distanceSq(const Vec2& p, const Segment& s) noexcept;

double distance(const Vec2& p, const Segment& s) noexcept;

double denormalizedDistance(const Line& l, const Vec2& p) noexcept;

double denormalizedDistance(const Vec2& p, const Line& l) noexcept;

double distance(const Line& l, const Vec2& p) noexcept;

double distance(const Vec2& p, const Line& l) noexcept;

}  // namespace truck::geom
