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

tf2::Quaternion QuaternionAverage(const std::vector<tf2::Quaternion>& quats) {
    cv::Mat q(quats.size(), 4, CV_64F);

    for (size_t i = 0; i < quats.size(); i++) {
        q.at<double>(i, 0) = quats[i].x();
        q.at<double>(i, 1) = quats[i].y();
        q.at<double>(i, 2) = quats[i].z();
        q.at<double>(i, 3) = quats[i].w();
    }

    cv::Mat w, u, vt;
    cv::SVD::compute(q.t() * q, w, u, vt);

    return tf2::Quaternion(
        vt.at<double>(0, 0), 
        vt.at<double>(0, 1),
        vt.at<double>(0, 2),
        vt.at<double>(0, 3)
    );
}

}
