#include "lidar_map/builder.h"

#include "geom/distance.h"
#include "common/exception.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>

namespace truck::lidar_map {

using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>>;
using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

namespace {

Eigen::Vector3f translationVectorIn3D(double x, double y, double z) {
    return Eigen::Vector3f(x, y, z);
}

Eigen::Vector3f translationVectorIn3DFromPose(const geom::Pose& pose) {
    return translationVectorIn3D(pose.pos.x, pose.pos.y, 0.0);
}

/**
 * Returns 3x3 rotation matrix by Z-axis by given theta rotation
 */
Eigen::Matrix3f rotationMatrixIn3D(double theta) {
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
    rotation_matrix(0, 0) = std::cos(theta);
    rotation_matrix(0, 1) = -1.0 * std::sin(theta);
    rotation_matrix(1, 0) = std::sin(theta);
    rotation_matrix(1, 1) = std::cos(theta);
    return rotation_matrix;
}

/**
 * Returns 3x3 rotation matrix by Z-axis by given pose
 */
Eigen::Matrix3f rotationMatrixIn3DFromPose(const geom::Pose& pose) {
    return rotationMatrixIn3D(pose.dir.angle().radians());
}

/**
 * Returns 3x3 transformation by given 2D translation vector (tx, ty) and rotation angle theta
 */
Eigen::Matrix3f transformationMatrix(const geom::Pose& pose) {
    const double dtheta = pose.dir.angle().radians();
    const double cos_dtheta = std::cos(dtheta);
    const double sin_dtheta = std::sin(dtheta);
    
    Eigen::Matrix3f tf_matrix = Eigen::Matrix3f::Identity();
    
    // Rotation
    tf_matrix(0, 0) = cos_dtheta;
    tf_matrix(0, 1) = -1.0 * sin_dtheta;
    tf_matrix(1, 0) = sin_dtheta;
    tf_matrix(1, 1) = cos_dtheta;

    // Translation
    tf_matrix(0, 2) = pose.pos.x;
    tf_matrix(1, 2) = pose.pos.y;

    return tf_matrix;
}

/**
 * Returns 3x3 transformation of 'pose_j' relatively to 'pose_i', knows as T_ij
 *
 * Poses 'pose_i' and 'pose_j' are given in a world frame,
 * so a 2D translation vector (tx, ty) of a 'pose_j' relatively to 'pose_i' CAN'T be expressed as:
 *
 * (tx, ty) = (pose_j.pos.x - pose_i.pos.x, pose_j.pos.y - pose_i.pos.y)
 *
 * 2D translation vector (tx, ty) is extracted from a 3x3 transformation T_ij, where:
 *
 * T_ij = T_iw * T_wj
 * T_wj: 3x3 transformation of 'pose_j' relatively to a world frame
 * T_wi: 3x3 transformation of 'pose_i' relatively to a world frame
 * T_iw = (T_wi).inverse(): 3x3 transformation of a world frame relatively to 'pose_i'
 *
 * In this function 2D translation vector (tx, ty) is calculated directly to avoid
 * inaccuracies of inverse operation
 */
Eigen::Matrix3f transformationMatrix(
    const geom::Pose& pose_i, const geom::Pose& pose_j) {
    const double theta_i = pose_i.dir.angle().radians();
    const double theta_j = pose_j.dir.angle().radians();
    const double dtheta = theta_j - theta_i;

    const double dx = pose_j.pos.x - pose_i.pos.x;
    const double dy = pose_j.pos.y - pose_i.pos.y;

    const double cos_theta_i = std::cos(theta_i);
    const double sin_theta_i = std::sin(theta_i);

    const double tx = dx * cos_theta_i + dy * sin_theta_i;
    const double ty = -1.0 * dx * sin_theta_i + dy * cos_theta_i;

    return transformationMatrix({geom::Vec2(tx, ty), geom::AngleVec2(geom::Angle(dtheta))});
}

/**
 * Returns g2o pose constructed from geom::Pose
 */
g2o::SE2 toSE2(const geom::Pose& pose) {
    return g2o::SE2(pose.pos.x, pose.pos.y, pose.dir.angle().radians());
}

/**
 * Returns g2o pose constructed from 3x3 transformation matrix in 2D
 */
g2o::SE2 toSE2(const Eigen::Matrix3f& tf_matrix) {
    const double tx = tf_matrix(0, 2);
    const double ty = tf_matrix(1, 2);
    const double dtheta = std::atan2(tf_matrix(1, 0), tf_matrix(0, 0));
    return g2o::SE2(tx, ty, dtheta);
}

/**
 * Returns geom::Pose constructed from g2o pose
 */
geom::Pose toPose(const g2o::SE2& se2) {
    return {
        {se2.translation().x(), se2.translation().y()}, {geom::AngleVec2(geom::Angle(se2.rotation().angle()))}};
}

}  // namespace

Builder::Builder(const BuilderParams& params, const ICP& icp) : params_(params), icp_(icp) {}

/**
 * Returns a subset of given 'poses' and 'clouds' via removing too close poses
 * Next pose will be added, if it's far enough from a previous one
 */
std::pair<geom::Poses, Clouds> Builder::filterByPosesProximity(
    const geom::Poses& poses, const Clouds& clouds) {
    VERIFY(poses.size() > 0);
    VERIFY(poses.size() == clouds.size());

    geom::Poses filtered_poses = {poses[0]};
    Clouds filtered_clouds = {clouds[0]};
    geom::Pose ref_pose = poses[0];

    for (size_t i = 1; i < poses.size(); i++) {
        if (geom::distance(poses[i].pos, ref_pose.pos) < params_.poses_min_dist) {
            continue;
        }

        filtered_poses.push_back(poses[i]);
        filtered_clouds.push_back(clouds[i]);
        ref_pose = poses[i];
    }

    return std::make_pair(filtered_poses, filtered_clouds);
}

/**
 * Input: set of 'poses' and corresponding 'clouds'
 * Points of cloud 'clouds[i]' should be in the frame of the corresponding pose 'poses[i]'
 *
 * Output: set of updated poses for given 'clouds'
 */
geom::Poses Builder::optimizePoses(const geom::Poses& poses, const Clouds& clouds) {
    g2o::SparseOptimizer optimizer;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    optimizer.setAlgorithm(solver);

    std::vector<g2o::VertexSE2*> vertices;

    // Add vertices
    for (size_t i = 0; i < poses.size(); i++) {
        auto* vertex = new g2o::VertexSE2();
        vertex->setId(i);
        vertex->setEstimate(toSE2(poses[i]));

        optimizer.addVertex(vertex);
        vertices.push_back(vertex);
    }

    // Add odometry edges
    for (size_t i = 1; i < poses.size(); i++) {
        Eigen::Matrix3f tf_matrix_odom = transformationMatrix(poses[i - 1], poses[i]);

        auto* edge = new g2o::EdgeSE2();
        edge->setVertex(0, vertices[i - 1]);
        edge->setVertex(1, vertices[i]);
        edge->setMeasurement(toSE2(tf_matrix_odom));
        edge->setInformation(Eigen::Matrix3d::Identity() * params_.odom_edge_weight);

        optimizer.addEdge(edge);
    }

    // Add ICP edges
    for (size_t i = 0; i < poses.size(); i++) {
        for (size_t j = i + 1; j < poses.size(); j++) {
            if (geom::distance(poses[i], poses[j]) > params_.icp_edge_max_dist) {
                continue;
            }

            Eigen::Matrix3f tf_matrix_odom = transformationMatrix(poses[i], poses[j]);

            Cloud cloud_i = Cloud(clouds[i]);
            Cloud cloud_j_tf = Cloud(clouds[j]);
            icp_.transformations.apply(cloud_j_tf, tf_matrix_odom);

            Eigen::Matrix3f tf_matrix_icp = icp_(cloud_j_tf, cloud_i);
            Eigen::Matrix3f tf_matrix_final = tf_matrix_icp * tf_matrix_odom;

            auto* edge = new g2o::EdgeSE2();
            edge->setVertex(0, vertices[i]);
            edge->setVertex(1, vertices[j]);
            edge->setMeasurement(toSE2(tf_matrix_final));
            edge->setInformation(Eigen::Matrix3d::Identity() * params_.icp_edge_weight);

            optimizer.addEdge(edge);
        }
    }

    g2o::VertexSE2* fixed_vertex = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(0));
    fixed_vertex->setFixed(true);

    optimizer.setVerbose(params_.verbose);

    optimizer.initializeOptimization();
    optimizer.optimize(params_.optimizer_steps);

    geom::Poses optimized_poses;

    for (size_t i = 0; i < poses.size(); i++) {
        g2o::VertexSE2* optimized_vertex = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(i));
        if (optimized_vertex) {
            g2o::SE2 tf_vector = optimized_vertex->estimate();
            optimized_poses.push_back(toPose(tf_vector));
        }
    }

    return optimized_poses;
}

/**
 * Input: set of 'poses' and corresponding 'clouds'
 * Points of cloud 'clouds[i]' should be in the frame of the corresponding pose 'poses[i]'
 *
 * Output: set of clouds, whose point coordinates are in the world frame
 */
Clouds Builder::transformClouds(const geom::Poses& poses, const Clouds& clouds) {
    VERIFY(poses.size() > 0);
    VERIFY(poses.size() == clouds.size());

    Clouds tf_clouds;

    for (size_t i = 0; i < clouds.size(); i++) {
        Eigen::Matrix3f rotation_matrix = rotationMatrixIn3DFromPose(poses[i]);
        Eigen::Vector3f translation_vector = translationVectorIn3DFromPose(poses[i]);

        Cloud tf_cloud = Cloud(clouds[i]);

        // Applying 'rotation_matrix' and 'translation_vector'
        // to each (x,y,z) point 'tf_cloud.features.col(j)' of curernt cloud
        for (size_t j = 0; j < tf_cloud.features.cols(); j++) {
            tf_cloud.features.col(j) =
                rotation_matrix * tf_cloud.features.col(j) + translation_vector;
        }

        tf_clouds.push_back(tf_cloud);
    }

    return tf_clouds;
}

Cloud Builder::mergeClouds(const Clouds& clouds) {
    VERIFY(clouds.size() > 0);
    Cloud merged_cloud = Cloud(clouds[0]);

    for (size_t i = 1; i < clouds.size(); i++) {
        merged_cloud.concatenate(clouds[i]);
    }

    return merged_cloud;
}

}  // namespace truck::lidar_map
