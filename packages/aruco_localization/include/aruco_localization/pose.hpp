#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace rosaruco {

struct Pose {
    tf2::Quaternion orientation;
    tf2::Vector3 point;
};

}
