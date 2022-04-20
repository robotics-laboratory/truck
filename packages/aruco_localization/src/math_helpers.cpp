#include "math_helpers.hpp"

tf2::Quaternion math_helpers::RotationVectorToQuaternion(const cv::Vec3d& rot_vec) {
    double angle = cv::norm(rot_vec);
    auto unit_vec = cv::normalize(rot_vec);
    return tf2::Quaternion(tf2::Vector3(unit_vec[0], unit_vec[1], unit_vec[2]), angle);
}

cv::Vec3d math_helpers::RotateUsingQuaternion(const cv::Vec3d& p, const tf2::Quaternion& rot_quat) {
    tf2::Vector3 p_tf2 = {p[0], p[1], p[2]};
    auto rotated_p_tf2 = tf2::quatRotate(rot_quat, p_tf2);
    return cv::Vec3d(rotated_p_tf2[0], rotated_p_tf2[1], rotated_p_tf2[2]);
}