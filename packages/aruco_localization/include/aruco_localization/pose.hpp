#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace robolab {
namespace aruco {

struct Pose {
    tf2::Quaternion orientation;
    tf2::Vector3 point;
};

}
}