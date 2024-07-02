#pragma once

#include <pointmatcher/PointMatcher.h>

namespace truck::lidar_map {

using Matcher = PointMatcher<float>;
using Cloud = Matcher::DataPoints;
using Clouds = std::vector<Cloud>;
using ICP = Matcher::ICP;
using TransformationParameters = Matcher::TransformationParameters;

}  // namespace truck::lidar_map
