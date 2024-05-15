#pragma once

#include <pointmatcher/PointMatcher.h>

namespace truck::icp_odometry {

using Matcher = PointMatcher<float>;

using DataPoints = Matcher::DataPoints;
using Matrix = Matcher::Matrix;
using TransformationParameters = Matcher::TransformationParameters;

}  // namespace truck::icp_odometry
