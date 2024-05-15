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

tf2::Vector3 ElementWiseMul(const tf2::Vector3& x, const tf2::Vector3& y);

}  // namespace rosaruco
