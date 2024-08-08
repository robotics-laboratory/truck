#pragma once

#include <pointmatcher/PointMatcher.h>

namespace truck::lidar_map {

using Matcher = PointMatcher<float>;
using ICP = Matcher::ICP;
using DataPoints = Matcher::DataPoints;

/**
 * 3xn eigen matrix of 2D point cloud in homogeneous coordinates
 */
using Cloud = Eigen::Matrix3Xf;

/**
 * 3xn eigen matrices of 2D point clouds in homogeneous coordinates
 */
using Clouds = std::vector<Cloud>;

}  // namespace truck::lidar_map
