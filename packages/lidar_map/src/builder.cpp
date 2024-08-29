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
        const auto scalar = cloud.col(i)(2);
        cloud.col(i) /= scalar;
    }
}

void normalize(DataPoints& data_points) {
    auto& matrix = data_points.features;
    for (size_t i = 0; i < matrix.cols(); i++) {
        const auto scalar = matrix.col(i)(2);
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
    const Eigen::Matrix3f T_wj = transformationMatrix(pose_j);
    const Eigen::Matrix3f T_wi = transformationMatrix(pose_i);
    return T_wi.inverse() * T_wj;
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
    const geom::Poses& poses, const Clouds& clouds, double poses_min_dist) const {
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

        if (dist < poses_min_dist) {
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

/**
 * Merge clouds column-wise
 */
Cloud Builder::mergeClouds(const Clouds& clouds) const {
    VERIFY(!clouds.empty());
    size_t points_count = 0;

    for (const auto& cloud : clouds) {
        points_count += cloud.cols();
    }

    Cloud merged_cloud(3, points_count);
    size_t last_point_id = 0;

    for (const auto& cloud : clouds) {
        merged_cloud.block(0, last_point_id, cloud.rows(), cloud.cols()) = cloud;
        last_point_id += cloud.cols();
    }

    return merged_cloud;
}

/**
 * Merge clouds column-wise with filtering by pose similarity
 *
 * Parameters:
 * - clouds: clouds which should be merged
 * - sim_points_max_dist: max dist for considering points similar to each other
 * - sim_points_min_count: min number of similar points relatively to the current point
 *   that must be found to include current point into the final cloud
 * - clouds_range: clouds range in which similar points are searched
 *
 * Description:
 * Loop through all clouds, for each j-th point of i-th cloud
 * we should find at least "sim_points_min_count" points among neighboring clouds of i-th cloud
 * which are less than "sim_points_max_dist" meters away from j-th point of i-th cloud
 *
 * Neighboring clouds of i-th cloud are in range [i - "clouds_range", i + "clouds_range")
 *
 * If "clouds_range" = -1, than neighboring clouds range is unlimited
 */
Cloud Builder::mergeCloudsByPointsSimilarity(
    const Clouds& clouds, int sim_points_min_count, double sim_points_max_dist,
    int clouds_range) const {
    VERIFY(!clouds.empty());

    auto get_data_point_by_id = [](const DataPoints& data_points, size_t id) {
        DataPoints data_point(data_points.createSimilarEmpty(1));
        data_point.features.col(0) = data_points.features.col(id);
        return data_point;
    };

    if (params_.verbose) {
        std::cout << "mergeCloudsByPointsSimilarity: init\n";
    }

    std::vector<std::shared_ptr<Matcher::Matcher>> kd_tree_arr;
    PointMatcherSupport::Parametrizable::Parameters kd_tree_params = {
        {"knn", "1"}, {"epsilon", "0"}};

    const auto data_points_arr = toDataPoints(clouds);

    for (const auto& data_points : data_points_arr) {
        auto kd_tree = Matcher::get().MatcherRegistrar.create("KDTreeMatcher", kd_tree_params);
        kd_tree->init(data_points);
        kd_tree_arr.push_back(kd_tree);
    }

    const int clouds_count = clouds.size();

    DataPoints data_points_filtered(data_points_arr[0].createSimilarEmpty());

    for (int cloud_id = 0; cloud_id < clouds_count; cloud_id++) {
        if (params_.verbose) {
            std::cout << "mergeCloudsByPointsSimilarity: " << cloud_id << " of " << clouds_count
                      << " iterations\n";
        }

        const int points_count = clouds[cloud_id].cols();

        for (int point_id = 0; point_id < points_count; point_id++) {
            int sim_points_cur_count = 0;

            const auto data_point = get_data_point_by_id(data_points_arr[cloud_id], point_id);

            const int start_neighbor_cloud_id =
                (clouds_range == -1) ? 0 : std::max(0, cloud_id - clouds_range);

            const int end_neighbor_cloud_id = (clouds_range == -1)
                                                  ? clouds_count
                                                  : std::min(clouds_count, cloud_id + clouds_range);

            for (int neighbor_cloud_id = start_neighbor_cloud_id;
                 neighbor_cloud_id < end_neighbor_cloud_id;
                 neighbor_cloud_id++) {
                if (sim_points_cur_count == sim_points_min_count) {
                    data_points_filtered.concatenate(data_point);
                    break;
                }

                const auto dist =
                    kd_tree_arr[neighbor_cloud_id]->findClosests(data_point).dists(0, 0);

                if (dist < sim_points_max_dist) {
                    sim_points_cur_count++;
                }
            }
        }
    }

    return data_points_filtered.features;
}

Clouds Builder::applyGridFilter(const Clouds& clouds, double cell_size) const {
    const std::string cell_size_str = std::to_string(cell_size);
    PointMatcherSupport::Parametrizable::Parameters grid_filter_params = {
        {"vSizeX", cell_size_str},
        {"vSizeY", cell_size_str},
        {"vSizeZ", cell_size_str},
    };

    std::shared_ptr<Matcher::DataPointsFilter> grid_filter =
        Matcher::get().DataPointsFilterRegistrar.create(
            "VoxelGridDataPointsFilter", grid_filter_params);

    Clouds clouds_filtered;

    for (const auto& cloud : clouds) {
        clouds_filtered.push_back(grid_filter->filter(toDataPoints(cloud)).features);
    }

    return clouds_filtered;
}

}  // namespace truck::lidar_map
