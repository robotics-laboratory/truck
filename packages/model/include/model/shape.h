#pragma once

#include "geom/polygon.h"
#include "geom/pose.h"

#include "yaml-cpp/yaml.h"

namespace truck::model {

struct Shape {
    Shape();
    Shape(const YAML::Node& node);

    double width;
    double length;
    double base_to_rear;
    int circles_count;

    double radius() const;
    std::vector<geom::Vec2> getCircleDecomposition(const geom::Pose& ego_pose) const;
    geom::Polygon rearPoseToShapePolygon(const geom::Pose rear_pose) const;
    geom::Polygon basePoseToShapePolygon(const geom::Pose base_pose, double base_to_rear) const;
};

}  // namespace truck::model
