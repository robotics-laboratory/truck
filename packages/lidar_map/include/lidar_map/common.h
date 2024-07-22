#pragma once

#include <pointmatcher/PointMatcher.h>

namespace truck::lidar_map {

using Matcher = PointMatcher<float>;
using ICP = Matcher::ICP;
using DataPoints = Matcher::DataPoints;

/**
 * 3xn matrix of 2D LiDAR points in homogeneous coordinates
 *
 * Cloud(0, i): x_i
 * Cloud(1, i): y_i
 * Cloud(2, i): 1.0
 */
using Cloud = Eigen::Matrix3Xf;

/**
 * Vector of 3xn matrices of 2D LiDAR points in homogeneous coordinates
 *
 * Clouds[i](0, j): x_j
 * Clouds[i](1, j): y_j
 * Clouds[i](2, j): 1.0
 */
using Clouds = std::vector<Cloud>;

}  // namespace truck::lidar_map
