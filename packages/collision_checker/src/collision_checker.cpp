#include "collision_checker/collision_checker.h"

namespace truck::collision_checker {

StaticCollisionChecker::StaticCollisionChecker(const model::Shape& shape) : shape_(shape), max_dist_(10.0) {}

const cv::Mat& StaticCollisionChecker::getDistanceTransform() const {
    return distance_transform_;
}

/**
 * Find the distance to the nearest obstacle
 * @param ego_pose car pose in 'base' frame
 * @return min distance
 */
double StaticCollisionChecker::operator()(const geom::Pose& ego_pose) const {   
    double min_dist = max_dist_;
    std::vector<geom::Vec2> points = shape_.getCircleDecomposition(ego_pose);

    for (const geom::Vec2& point : points) {
        min_dist = std::min(min_dist, getDistance(point));
    }

    return min_dist;
}

/**
 * Update information about grid and calculate distance transform matrix
 * @param grid scanned occupancy grid
 */
void StaticCollisionChecker::reset(const nav_msgs::msg::OccupancyGrid& grid) {
    grid_metadata_ = grid.info;

    // convert grid position
    tf2::Vector3 grid_pos_tf2(
        grid_metadata_.origin.position.x,
        grid_metadata_.origin.position.y,
        grid_metadata_.origin.position.z
    );

    // convert grid orientation
    tf2::Quaternion grid_quat_tf2;
    tf2::fromMsg(grid_metadata_.origin.orientation, grid_quat_tf2);

    // initialize transformation from 'grid' frame to 'base' frame
    transform_from_grid_to_base_ = tf2::Transform(grid_quat_tf2, grid_pos_tf2);

    // initialize binary grid matrix
    cv::Mat binary_grid = cv::Mat(grid_metadata_.height, grid_metadata_.width, CV_8UC1);

    // fill binary grid matrix with values based on occupancy grid values
    for (int i = 0; i < grid.info.height; i++) {
        for (int j = 0; j < grid_metadata_.width; j++) {
            int8_t grid_cell = grid.data.at(i * grid_metadata_.width + j);

            binary_grid.at<uchar>(i, j) =
                grid_cell == 0
                    ? 1
                    : 0;
        }
    }

    // calculate distance transform matrix
    cv::distanceTransform(binary_grid, distance_transform_, cv::DIST_L2, cv::DIST_MASK_PRECISE);
}

double StaticCollisionChecker::getDistance(const geom::Vec2& point) const {
    // conver point in 'base' frame from 'geom::Vec2' type to 'tf2::Vector3' type
    tf2::Vector3 point_tf2 = tf2::Vector3(
        point.x,
        point.y,
        0.0
    );

    // get point in 'grid' frame with 'tf2::Vector3' type
    tf2::Vector3 grid_point_tf2 = transform_from_grid_to_base_.inverse()(point_tf2);

    // find relevant indices of distance transform matrix
    int width_index = floor<int>(grid_point_tf2.x() / grid_metadata_.resolution);
    int height_index = floor<int>(grid_point_tf2.y() / grid_metadata_.resolution);

    // check borders
    if ((width_index >= grid_metadata_.width) ||
        (height_index >= grid_metadata_.height) ||
        (width_index < 0) ||
        (height_index < 0)) {

        return max_dist_;
    }

    const double distance =
        distance_transform_.at<float>(height_index, width_index) * grid_metadata_.resolution;

    return std::max(distance - shape_.radius(), 0.0);
}

} // namespace truck::collision_checker