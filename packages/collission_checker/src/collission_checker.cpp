#include "collission_checker/collission_checker.h"

StaticCollisionChecker::StaticCollisionChecker(const Shape& shape) : shape_() {
    shape_ = shape;
}

void StaticCollisionChecker::reset(const nav_msgs::msg::OccupancyGrid& grid) {
    resolution_ = grid.info.resolution;
    width_ = grid.info.width;
    height_ = grid.info.height;

    cv::Mat binary_grid = cv::Mat(height_, width_, CV_8UC1);

    for (int i = 0; i < height_; i++) {
        for (int j = 0; j < width_; j++) {
            auto grid_cell = int(grid.data.at(i * height_ + j));

            binary_grid.at<uchar>(i, j) =
                grid_cell == 0
                    ? 1
                    : 0;
        }
    }
    
    cv::distanceTransform(binary_grid, distance_transform_, cv::DIST_L2, cv::DIST_MASK_PRECISE);
}

double StaticCollisionChecker::operator() (const geom::Pose& ego_pose) const {
    double min_dist = std::numeric_limits<double>::max();
    std::vector<geom::Vec2> points = shape_.getCircleDecomposition(ego_pose);

    for (const geom::Vec2& point : points) {
        double current_dist = getDistance(point);
        min_dist = std::min(min_dist, current_dist);
    }

    return min_dist;
}

// cv::Mat findDistanceTransform() const {}

double StaticCollisionChecker::getDistance(const geom::Vec2 point) const {
    double dist;
    int width_index = floor<int>(point.x / resolution_);
    int height_index = floor<int>(point.y / resolution_);

    if ((width_index >= width_) ||
        (height_index >= height_) ||
        (width_index < 0) ||
        (height_index < 0)) {

        return std::numeric_limits<double>::max();
    }

    dist = distance_transform_.at<float>(height_index, width_index);
    dist = (dist * resolution_) - shape_.radius();

    return dist > 0 ? dist : 0;
}