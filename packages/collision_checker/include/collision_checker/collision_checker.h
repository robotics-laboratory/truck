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
        nav_msgs::msg::MapMetaData grid_metadata;

        double getDistance(const geom::Vec2& ekf_point) const;
};

}