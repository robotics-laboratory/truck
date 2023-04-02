#pragma once

#include "geom/pose.h"

namespace truck::geom {

struct Localization {
    Pose pose;
    double velocity;
};

}  // namespace truck::geom