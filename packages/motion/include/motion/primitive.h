#pragma once

#include "geom/pose.h"

namespace truck::geom {

Poses findMotion(const Pose& from, const Vec2& to, double gamma, double step);

Poses findMotion(const Pose& from, const Pose& to, double gamma, double step);

}  // namespace truck::geom
