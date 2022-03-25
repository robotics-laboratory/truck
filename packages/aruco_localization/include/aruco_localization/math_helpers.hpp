#include <opencv2/core.hpp>
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>

namespace MathHelpers {

    tf2::Quaternion RotationVectorToQuaternion(const cv::Vec3d& rot_vec);
    
    cv::Vec3d RotateUsingQuaternion(const cv::Vec3d& p, const tf2::Quaternion& rot_quat);
}