#pragma once

#include "model/params.h"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2/impl/utils.h>

#include <cmath>
#include <vector>
#include <limits>

namespace truck::collision_checker {

class StaticCollisionChecker {
    public:
        const cv::Mat& getDistanceTransform() const;
        double operator() (const geom::Pose& ego_pose) const;
        void reset(
            const nav_msgs::msg::OccupancyGrid& grid,
            const tf2::Vector3& transform_translation,
            const tf2::Quaternion& transform_quat
        );

        StaticCollisionChecker(const model::Shape& shape);

    private:
        double max_dist_;        
        float resolution_;
        model::Shape shape_;
        uint32_t width_, height_;
        cv::Mat distance_transform_;
        tf2::Quaternion transform_quat_;
        tf2::Vector3 transform_translation_;
        
        double getDistance(const geom::Vec2& ekf_point) const;
};

}