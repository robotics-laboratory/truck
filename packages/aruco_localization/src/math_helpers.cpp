#include "math_helpers.hpp"

tf2::Quaternion MathHelpers::RotationVectorToQuaternion(const cv::Vec3d& rot_vec) {
    double angle = cv::norm(rot_vec);
    auto unit_vec = cv::normalize(rot_vec);
    return tf2::Quaternion(tf2::Vector3(rot_vec[0], rot_vec[1], rot_vec[2]), angle);
}

cv::Vec3d MathHelpers::RotateUsingQuaternion(const cv::Vec3d& p, tf2::Quaternion& rot_quat) {
    tf2::Quaternion p_quat = tf2::Quaternion(p[0], p[1], p[2], 0),
        res_quat = rot_quat * p_quat * rot_quat.inverse();
    return cv::Vec3d(res_quat[0], res_quat[1], res_quat[2]);
}