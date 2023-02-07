#pragma once

#include "model/params.h"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2/impl/utils.h>

#include <cmath>
#include <vector>
#include <limits>

namespace truck::collision_checker {

/**
 * StaticCollisionChecker library
 * @attention occupancy grid and query points must be in the same coordinate system
 */
class StaticCollisionChecker {
    public:
        const cv::Mat& getDistanceTransform() const;
        double operator()(const geom::Pose& ego_pose) const;
        void reset(const nav_msgs::msg::OccupancyGrid& grid);
        
        StaticCollisionChecker(const model::Shape& shape);

    private:
        double max_dist_;        
        model::Shape shape_;
        cv::Mat distance_transform_;
        nav_msgs::msg::MapMetaData grid_metadata_;
        tf2::Transform transform_from_grid_to_base_;

        double getDistance(const geom::Vec2& point) const;
};

} // namespace truck::collision_checker