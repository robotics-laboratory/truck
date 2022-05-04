#include "math/math_helpers.hpp"

namespace rosaruco {

tf2::Quaternion RotationVectorToQuaternion(const cv::Vec3d& rot_vec) {
    double angle = cv::norm(rot_vec);
    auto axis = cv::normalize(rot_vec);
    return tf2::Quaternion(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
}

Transform GetTransform(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    return Transform(RotationVectorToQuaternion(rvec), {tvec[0], tvec[1], tvec[2]});
}

}
