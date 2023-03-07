#include "collision_checker/util.h"

#include <geom/msg.h>

#include <opencv2/imgproc.hpp>

namespace truck::collision_checker {

cv::Mat distanceTransform(const nav_msgs::msg::OccupancyGrid& map) {
    // initialize binary grid matrix
    cv::Mat binary_grid = cv::Mat(map.info.height, map.info.width, CV_8UC1);

    // fill binary grid matrix with values based on occupancy grid values
    for (uint32_t i = 0; i < map.info.height; i++) {
        for (uint32_t j = 0; j < map.info.width; j++) {
            int8_t grid_cell = map.data.at(i * map.info.width + j);
            binary_grid.at<uchar>(i, j) = grid_cell == 0 ? 1 : 0;
        }
    }

    cv::Mat result;
    cv::distanceTransform(binary_grid, result, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    return result;
}

MapMeta toMapMetadata(const nav_msgs::msg::MapMetaData& info) {
    return {
        .origin = geom::toPose(info.origin),
        .resolution = info.resolution,
        .width = info.width,
        .height = info.height
    };
}

}  // namespace truck::collision_checker
