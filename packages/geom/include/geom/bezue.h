#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::geom {

Poses bezue1(const Vec2& p0, const Vec2& p1, size_t n);
Poses bezue1(const Vec2& p0, const Vec2& p1, double step);

Poses bezue2(const Vec2& p0, const Vec2& p1, const Vec2& p2, size_t n);
Poses bezue2(const Vec2& p0, const Vec2& p1, const Vec2& p2, double step);

Poses bezue3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n);
Poses bezue3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double step);

} // namespace truck::geom