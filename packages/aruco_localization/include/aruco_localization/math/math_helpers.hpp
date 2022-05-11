#pragma once

#include <cstring>
#include <vector>

#include <opencv2/core.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include "transform.hpp"

namespace rosaruco {
    
    tf2::Quaternion RotationVectorToQuaternion(const cv::Vec3d& rot_vec);
    Transform GetTransform(const cv::Vec3d& rvec, const cv::Vec3d& tvec);
    tf2::Quaternion QuaternionAverage(const std::vector<tf2::Quaternion>& quats);

}
