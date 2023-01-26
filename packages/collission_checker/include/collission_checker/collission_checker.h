#pragma once

#include "model/params.h"

#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cmath>
#include <vector>
#include <limits>

using namespace truck;
using namespace truck::model;

class StaticCollisionChecker {
    public:
        float resolution_;
        uint32_t width_, height_;
        cv::Mat distance_transform_;

        void reset(const nav_msgs::msg::OccupancyGrid& grid);
        double operator() (const geom::Pose& ego_pose) const;

        StaticCollisionChecker(const Shape& shape);

    private:
        Shape shape_;
        
        // cv::Mat findDistanceTransform() const;
        double getDistance(const geom::Vec2 point) const;
};