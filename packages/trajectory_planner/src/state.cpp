#include "trajectory_planner/state.h"

#include "common/exception.h"

#include "geom/bezier.h"
#include "geom/distance.h"

#include "motion/primitive.h"

namespace truck::trajectory_planner {

geom::Poses FindMotion(const geom::Pose& from, const geom::Pose& to, size_t max_step, double eps) {
    const auto dist = geom::distance(from.pos, to.pos);

    if (dist < eps && std::abs(from.dir.angle().radians() - to.dir.angle().radians()) < eps) {
        return geom::Poses({from});
    }

    if (dist < eps) {
        return geom::Poses();
    }

    const double gamma = dist * 0.5;
    const geom::Vec2 from_ref = from.pos + from.dir.vec() * gamma;
    const geom::Vec2 to_ref = to.pos - to.dir.vec() * gamma;

    return bezier3(from.pos, from_ref, to_ref, to.pos, max_step);
}

double MotionLength(const geom::Poses& motion, double inf) {
    if (motion.empty()) {
        return inf;
    }

    double motion_lenght = 0;
    for (auto it = motion.begin(); it + 1 != motion.end(); ++it) {
        motion_lenght += geom::distance(it->pos, (it + 1)->pos);
    }

    return motion_lenght;
}

double MotionTime(
    double motion_length, double form_velocity, double to_velocity, double eps, double inf) {
    if (motion_length < eps && std::abs(form_velocity - to_velocity) < eps) {
        return 0;
    }

    const auto av_velocity = (form_velocity + to_velocity) * 0.5;

    if (motion_length < eps || std::abs(av_velocity) < eps) {
        return inf;
    }

    return motion_length / av_velocity;
}

};  // namespace truck::trajectory_planner