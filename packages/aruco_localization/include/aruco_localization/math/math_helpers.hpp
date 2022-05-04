#pragma once

#include <opencv2/core.hpp>
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include "transform.hpp"

namespace rosaruco {

    tf2::Quaternion RotationVectorToQuaternion(const cv::Vec3d& rot_vec);
    Transform GetTransform(const cv::Vec3d& rvec, const cv::Vec3d& tvec);

}