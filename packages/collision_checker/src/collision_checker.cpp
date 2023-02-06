#include "collision_checker/collision_checker.h"

namespace truck::collision_checker {

StaticCollisionChecker::StaticCollisionChecker(const model::Shape& shape) : shape_(shape), max_dist_(10.0) {}

const cv::Mat& StaticCollisionChecker::getDistanceTransform() const {
    return distance_transform_;
}

double StaticCollisionChecker::operator()(const geom::Pose& ego_pose) const {   
    // input: 'ego_pose' - car pose in 'base' frame

    double min_dist = max_dist_;
    std::vector<geom::Vec2> points = shape_.getCircleDecomposition(ego_pose);

    for (const geom::Vec2& point : points) {
        double current_dist = getDistance(point);
        min_dist = std::min(min_dist, current_dist);
    }

    return min_dist;
}

void StaticCollisionChecker::reset(const nav_msgs::msg::OccupancyGrid& grid) {
    grid_metadata = grid.info;
    cv::Mat binary_grid = cv::Mat(grid_metadata.height, grid_metadata.width, CV_8UC1);

    for (int i = 0; i < grid.info.height; i++) {
        for (int j = 0; j < grid_metadata.width; j++) {
            int8_t grid_cell = grid.data.at(i * grid_metadata.width + j);

            binary_grid.at<uchar>(i, j) =
                grid_cell == 0
                    ? 1
                    : 0;
        }
    }

    cv::distanceTransform(binary_grid, distance_transform_, cv::DIST_L2, cv::DIST_MASK_PRECISE);
}

double StaticCollisionChecker::getDistance(const geom::Vec2& base_point) const {
    // Сonvert grid orientation
    tf2::Quaternion grid_quat_tf2;
    tf2::fromMsg(grid_metadata.origin.orientation, grid_quat_tf2);

    // Сonvert grid position
    tf2::Vector3 grid_pos_tf2(
        grid_metadata.origin.position.x,
        grid_metadata.origin.position.y,
        grid_metadata.origin.position.z
    );
    
    // Initialize tranformation from 'grid' frame to 'base' grid
    tf2::Transform transform_from_grid_to_base(grid_quat_tf2, grid_pos_tf2);

    // Conver point in 'base' frame from 'geom::Vec2' to 'tf2::Vector3' type
    tf2::Vector3 base_point_tf2 = tf2::Vector3(
        base_point.x,
        base_point.y,
        0.0
    );

    // Get point in 'grid' frame
    tf2::Vector3 grid_point_tf2 = transform_from_grid_to_base.inverse()(base_point_tf2);

    int width_index = floor<int>(grid_point_tf2.x() / grid_metadata.resolution);
    int height_index = floor<int>(grid_point_tf2.y() / grid_metadata.resolution);

    if ((width_index >= grid_metadata.width) ||
        (height_index >= grid_metadata.height) ||
        (width_index < 0) ||
        (height_index < 0)) {

        return max_dist_;
    }

    const double distance =
        distance_transform_.at<float>(height_index, width_index) * grid_metadata.resolution;

    return std::max(distance - shape_.radius(), 0.0);
}

}