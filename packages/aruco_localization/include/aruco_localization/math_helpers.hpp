#include <opencv2/core.hpp>
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>


namespace robolab {
namespace aruco {
namespace math_helpers {

    tf2::Quaternion RotationVectorToQuaternion(const cv::Vec3d& rot_vec);
    cv::Vec3d RotateUsingQuaternion(const cv::Vec3d& p, const tf2::Quaternion& rot_quat);
    tf2::Transform GetTransform(const cv::Vec3d& rvec, const cv::Vec3d& tvec);

}
}
}