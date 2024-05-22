#include "model/shape.h"

#include "geom/vector.h"

#include <boost/assert.hpp>

namespace truck::model {

Shape::Shape() {}

Shape::Shape(const YAML::Node& node) :
    width(node["width"].as<double>()),
    length(node["length"].as<double>()),
    base_to_rear(node["base_to_rear"].as<double>()),
    circles_count(node["circles_count"].as<int>()) {
    BOOST_VERIFY(width > 0);
    BOOST_VERIFY(length > 0);
    BOOST_VERIFY(base_to_rear > 0);
    BOOST_VERIFY(length > base_to_rear);
    BOOST_VERIFY(circles_count * 2 * radius() > length);
}

double Shape::radius() const { return width / 2; }

std::vector<geom::Vec2> Shape::getCircleDecomposition(const geom::Pose& ego_pose) const {
    std::vector<geom::Vec2> points;
    const double pos_first = -base_to_rear + radius();
    const double pos_step = (length - 2 * radius()) / (circles_count - 1);

    for (int i = 0; i < circles_count; i++) {
        double offset = pos_first + (i * pos_step);
        points.push_back(ego_pose.pos + offset * ego_pose.dir);
    }

    return points;
}

geom::Polygon Shape::rearPoseToShapePolygon(const geom::Pose rear_pose) const {
    const auto x = rear_pose.pos.x;
    const auto y = rear_pose.pos.y;
    const auto yaw = rear_pose.dir.vec();

    const auto dir = length * yaw;
    const auto norm = width / 2 * yaw;
    const geom::Vec2 a(x - norm.y, y + norm.x);
    const geom::Vec2 b(x + norm.y, y - norm.x);
    return {a, a + dir, b + dir, b};
}

geom::Polygon Shape::basePoseToShapePolygon(const geom::Pose base_pose) const {
    const geom::Pose rear_pose = {base_pose.pos - base_to_rear * base_pose.dir, base_pose.dir};
    return rearPoseToShapePolygon(rear_pose);
}

}  // namespace truck::model
