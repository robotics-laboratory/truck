#include "collision_checker/collision_checker.h"

namespace truck::collision_checker {

StaticCollisionChecker::StaticCollisionChecker(const model::Shape& shape) : shape_(shape), max_dist_(10.0) {}

const cv::Mat& StaticCollisionChecker::getDistanceTransform() const {
    return distance_transform_;
}

double StaticCollisionChecker::operator() (const geom::Pose& ego_pose) const {        
    double min_dist = max_dist_;
    std::vector<geom::Vec2> points = shape_.getCircleDecomposition(ego_pose);

    for (const geom::Vec2& point : points) {
        double current_dist = getDistance(point);
        min_dist = std::min(min_dist, current_dist);
    }

    return min_dist;
}

void StaticCollisionChecker::reset(
    const nav_msgs::msg::OccupancyGrid& grid,
    const tf2::Vector3& transform_translation,
    const tf2::Quaternion& transform_quat
    ) {
    width_ = grid.info.width;
    height_ = grid.info.height;
    resolution_ = grid.info.resolution;
    transform_quat_ = transform_quat;
    transform_translation_ = transform_translation;

    cv::Mat binary_grid = cv::Mat(height_, width_, CV_8UC1);

    for (int i = 0; i < height_; i++) {
        for (int j = 0; j < width_; j++) {
            int8_t grid_cell = grid.data.at(i * width_ + j);

            binary_grid.at<uchar>(i, j) =
                grid_cell == 0
                    ? 1
                    : 0;
        }
    }

    cv::distanceTransform(binary_grid, distance_transform_, cv::DIST_L2, cv::DIST_MASK_PRECISE);
}

double StaticCollisionChecker::getDistance(const geom::Vec2& ekf_point) const {
    tf2::Transform base_transform(transform_quat_, transform_translation_);

    tf2::Vector3 tf2_ekf_point(ekf_point.x, ekf_point.y, 0.0);
    tf2::Vector3 tf2_base_point = base_transform(tf2_ekf_point);
    
    int width_index = floor<int>(tf2_base_point.x() / resolution_);
    int height_index = floor<int>(tf2_base_point.y() / resolution_);

    if ((width_index >= width_) ||
        (height_index >= height_) ||
        (width_index < 0) ||
        (height_index < 0)) {

        return max_dist_;
    }

    const double distance =
        distance_transform_.at<float>(height_index, width_index) * resolution_;

    return std::max(distance - shape_.radius(), 0.0);
}

}