#include "motion/primitive.h"
#include "geom/bezier.h"

namespace truck::geom {

Poses findMotion(const Pose& from, const Vec2& to, double gamma, double step) {
    const Vec2 middle = from.pos + from.dir * gamma;
    return toPoses(bezier2(from.pos, middle, to, step));
}

Poses findMotion(const Pose& from, const Pose& to, double gamma, double step) {
    const Vec2 from_ref = from.pos + from.dir * gamma;
    const Vec2 to_ref = to.pos - to.dir * gamma;
    return toPoses(bezier3(from.pos, from_ref, to_ref, to.pos, step));
}

}  // namespace truck::geom
