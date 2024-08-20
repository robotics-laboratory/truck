#include "lidar_map/builder.h"

#include "common/exception.h"
#include "geom/boost/point.h"
#include "geom/distance.h"

#include <boost/geometry.hpp>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <fstream>

namespace truck::lidar_map {

namespace bg = boost::geometry;

using RTree = bg::index::rtree<geom::Vec2, bg::index::rstar<16>>;

using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>>;
using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

namespace {

void normalize(Cloud& cloud) {
    for (size_t i = 0; i < cloud.cols(); i++) {
        const double scalar = cloud.col(i)(2);
        cloud.col(i) /= scalar;
    }
}

void normalize(DataPoints& data_points) {
    auto& matrix = data_points.features;
    for (size_t i = 0; i < matrix.cols(); i++) {
        const double scalar = matrix.col(i)(2);
        matrix.col(i) /= scalar;
    }
}

DataPoints toDataPoints(const Cloud& cloud) {
    DataPoints::Labels feature_labels;
    feature_labels.push_back(DataPoints::Label("x", 1));
    feature_labels.push_back(DataPoints::Label("y", 1));
    feature_labels.push_back(DataPoints::Label("w", 1));

    DataPoints::Labels descriptor_labels;

    DataPoints data_points(feature_labels, descriptor_labels, cloud.cols());
    data_points.features = cloud;
    return data_points;
}

std::vector<DataPoints> toDataPoints(const Clouds& clouds) {
    std::vector<DataPoints> data_points_array;

    for (const auto& cloud : clouds) {
        data_points_array.push_back(toDataPoints(cloud));
    }

    return data_points_array;
}

/**
 * Returns g2o::SE2 constructed from pose
 */
g2o::SE2 toSE2(const geom::Pose& pose) {
    return {pose.pos.x, pose.pos.y, pose.dir.angle().radians()};
}

/**
 * Returns g2o::SE2 constructed from 3x3 transformation matrix
 */
g2o::SE2 toSE2(const Eigen::Matrix3f& tf_matrix) {
    const double tx = tf_matrix(0, 2);
    const double ty = tf_matrix(1, 2);
    const double dtheta = std::atan2(tf_matrix(1, 0), tf_matrix(0, 0));
    return {tx, ty, dtheta};
}

/**
 * Returns pose constructed from g2o::SE2
 */
geom::Pose toPose(const g2o::SE2& se2) {
    return {
        {se2.translation().x(), se2.translation().y()},
        {geom::AngleVec2::fromRadians(se2.rotation().angle())}};
}

/**
 * Returns 3x3 transformation matrix of pose relatively to a world
 *
 * Pose is given in a world frame
 *
 * Translation is taken from pose.pos
 * Rotation is taken from pose.dir
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
 * Returns 3x3 transformation matrix of pose_j relatively to pose_i (T_ij)
 *
 * Poses pose_i and pose_j are given in a world frame
 *
 * Translation and rotation is taken from 3x3 transformation matrix T_ij where:
 * - T_ij = T_iw * T_wj
 * - T_wj: 3x3 transformation matrix of pose_j relatively to a world
 * - T_wi: 3x3 transformation matrix of pose_i relatively to a world
 * - T_iw = (T_wi).inv(): 3x3 transformation matrix of world relatively to pose_i
 */
Eigen::Matrix3f transformationMatrix(const geom::Pose& pose_i, const geom::Pose& pose_j) {
    const double theta_i = pose_i.dir.angle().radians();
    const double theta_j = pose_j.dir.angle().radians();
    const double dtheta = theta_j - theta_i;

    const double dx = pose_j.pos.x - pose_i.pos.x;
    const double dy = pose_j.pos.y - pose_i.pos.y;

    const double cos_theta_i = std::cos(theta_i);
    const double sin_theta_i = std::sin(theta_i);

    const double tx = dx * cos_theta_i + dy * sin_theta_i;
    const double ty = -1.0 * dx * sin_theta_i + dy * cos_theta_i;

    return transformationMatrix({geom::Vec2(tx, ty), geom::AngleVec2::fromRadians(dtheta)});
}

}  // namespace

Builder::Builder(const BuilderParams& params) : params_(params) {
    std::ifstream icp_config_stream(params_.icp_config);
    icp_.loadFromYaml(icp_config_stream);
}

/**
 * Returns a subset of given poses and clouds via removing too close poses
 * Next pose will be added, if it's far enough from a previous one
 */
std::pair<geom::Poses, Clouds> Builder::filterByPosesProximity(
    const geom::Poses& poses, const Clouds& clouds) const {
    VERIFY(!poses.empty());
    VERIFY(poses.size() == clouds.size());

    geom::Poses filtered_poses;
    Clouds filtered_clouds;

    RTree rtree;
    rtree.insert(poses[0].pos);

    for (size_t i = 1; i < poses.size(); i++) {
        const auto& pose = poses[i];
        const auto& cloud = clouds[i];

        std::vector<geom::Vec2> query_points;
        rtree.query(bg::index::nearest(pose.pos, 1), std::back_inserter(query_points));

        const double dist = geom::distance(pose.pos, query_points.front());

        if (dist < params_.poses_min_dist) {
            continue;
        }

        filtered_poses.push_back(pose);
        filtered_clouds.push_back(cloud);

        rtree.insert(pose.pos);
    }

    return {filtered_poses, filtered_clouds};
}

/**
 * Optimize clouds' poses via optimization
 *
 * Input:
 * - set of poses in a world frame
 * - set of clouds which located in correspondig poses
 * - points coordinates of each cloud (clouds[i]) are given in a corresponding frame (poses[i])
 *
 * Output:
 * - set of optimized poses in a world frame
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
        const Eigen::Matrix3f tf_matrix_odom = transformationMatrix(poses[i - 1], poses[i]);

        auto* edge = new g2o::EdgeSE2();
        edge->setVertex(0, vertices[i - 1]);
        edge->setVertex(1, vertices[i]);
        edge->setMeasurement(toSE2(tf_matrix_odom));
        edge->setInformation(Eigen::Matrix3d::Identity() * params_.odom_edge_weight);

        optimizer.addEdge(edge);
    }

    auto data_points_clouds = toDataPoints(clouds);

    // Add ICP edges
    for (size_t i = 0; i < poses.size(); i++) {
        for (size_t j = i + 1; j < poses.size(); j++) {
            if (geom::distance(poses[i], poses[j]) > params_.icp_edge_max_dist) {
                continue;
            }

            const Eigen::Matrix3f tf_matrix_odom = transformationMatrix(poses[i], poses[j]);

            const auto& reference_cloud = data_points_clouds[i];

            auto reading_cloud = data_points_clouds[j];
            icp_.transformations.apply(reading_cloud, tf_matrix_odom);
            normalize(reading_cloud);

            const Eigen::Matrix3f tf_matrix_icp = icp_(reading_cloud, reference_cloud);
            const Eigen::Matrix3f tf_matrix_final = tf_matrix_icp * tf_matrix_odom;

            auto* edge = new g2o::EdgeSE2();
            edge->setVertex(0, vertices[i]);
            edge->setVertex(1, vertices[j]);
            edge->setMeasurement(toSE2(tf_matrix_final));
            edge->setInformation(Eigen::Matrix3d::Identity() * params_.icp_edge_weight);

            optimizer.addEdge(edge);
        }
    }

    auto* fixed_vertex = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(0));
    fixed_vertex->setFixed(true);

    optimizer.setVerbose(params_.verbose);

    optimizer.initializeOptimization();
    optimizer.optimize(params_.optimizer_steps);

    geom::Poses optimized_poses;

    for (size_t i = 0; i < poses.size(); i++) {
        auto* optimized_vertex = dynamic_cast<g2o::VertexSE2*>(optimizer.vertex(i));
        if (optimized_vertex) {
            const g2o::SE2 se2 = optimized_vertex->estimate();
            optimized_poses.push_back(toPose(se2));
        }
    }

    return optimized_poses;
}

/**
 * Transform points coordinates of each cloud from corresponding local frame to a common world frame
 *
 * Input:
 * - set of poses in a world frame
 * - set of clouds which located in correspondig poses
 * - points coordinates of each cloud (clouds[i]) are given in a corresponding frame (poses[i])
 *
 * Output:
 * - set of clouds in a world frame
 */
Clouds Builder::transformClouds(const geom::Poses& poses, const Clouds& clouds) const {
    VERIFY(!poses.empty());
    VERIFY(poses.size() == clouds.size());

    Clouds clouds_tf;

    for (size_t i = 0; i < clouds.size(); i++) {
        const Eigen::Matrix3f tf_matrix = transformationMatrix(poses[i]);

        Cloud cloud_tf = tf_matrix * clouds[i];
        normalize(cloud_tf);

        clouds_tf.push_back(cloud_tf);
    }

    return clouds_tf;
}

Cloud Builder::mergeClouds(const Clouds& clouds) const {
    VERIFY(!clouds.empty());
    Cloud merged_cloud = clouds[0];

    for (size_t i = 1; i < clouds.size(); i++) {
        Cloud tmp = Cloud(3, merged_cloud.cols() + clouds[i].cols());
        tmp << merged_cloud, clouds[i];
        merged_cloud = tmp;
    }

    return merged_cloud;
}

}  // namespace truck::lidar_map
