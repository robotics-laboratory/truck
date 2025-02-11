#pragma once

#include <pointmatcher/PointMatcher.h>

namespace truck::lidar_map {

using Matcher = PointMatcher<float>;
using ICP = Matcher::ICP;
using DataPoints = Matcher::DataPoints;

/**
 * 4xn eigen matrix of 3D point cloud in homogeneous coordinates
 */
using Cloud = Eigen::Matrix4Xf;

/**
 * 4xn eigen matrices of 3D point clouds in homogeneous coordinates
 */
using Clouds = std::vector<Cloud>;

struct CloudAttributes {
    Eigen::Matrix3Xf normals;
    Eigen::Matrix4Xf outliers;
};

struct CloudWithAttributes {
    Cloud cloud;
    CloudAttributes attributes;
};

}  // namespace truck::lidar_map
