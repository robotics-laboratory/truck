#include "collision/map.h"

#include "common/math.h"
#include "geom/msg.h"

#include <opencv2/imgproc.hpp>

namespace truck::collision {

Map Map::emptyLikeThis() const {
    return Map {
        .origin = origin,
        .resolution = resolution,
        .size = size,
        .data = cv::Mat()
    };
}

Map distanceTransform(const Map& map) {
    // initialize binary grid matrix
    auto result = map.emptyLikeThis();

    cv::distanceTransform(map.data, result.data, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    return result;
}

Map Map::fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map) {
    cv::Mat grid = cv::Mat(map.info.height, map.info.width, CV_8UC1);

    // fill binary grid matrix with values based on occupancy grid values
    for (uint32_t i = 0; i < map.info.height; i++) {
        for (uint32_t j = 0; j < map.info.width; j++) {
            int8_t grid_cell = map.data.at(i * map.info.width + j);
            grid.at<uchar>(i, j) = grid_cell == 0 ? 1 : 0;
        }
    }

    return Map {
        .origin = geom::toPose(map.info.origin),
        .resolution = map.info.resolution,
        .size = {
            static_cast<int>(map.info.width),
            static_cast<int>(map.info.height)
        },
        .data = grid
    };
}

nav_msgs::msg::OccupancyGrid Map::makeCostMap(
        const std_msgs::msg::Header& header,
        double kMaxDist) const {
    nav_msgs::msg::OccupancyGrid msg;

    const double max = kMaxDist / resolution;

    msg.header = header;
    msg.info.resolution = resolution;
    msg.info.width = size.width;
    msg.info.height = size.height;
    msg.info.origin = geom::msg::toPose(origin);

    msg.data = std::vector<int8_t>(size.width * size.height);


    for (int i = 0; i < size.height; i++) {
        for (int j = 0; j < size.width; j++) {
            const double cost = 1 - Limits(0., 1.).clamp(data.at<float>(i, j) / max);
            msg.data.at(i * size.width + j) = 100 * cost;
        }
    }

    return msg;
}

}  // namespace truck::collision
