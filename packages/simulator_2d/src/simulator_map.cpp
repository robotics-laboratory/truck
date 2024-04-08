#include "simulator_2d/simulator_map.h"

#include "map/map.h"
#include "geom/distance.h"
#include "geom/intersection.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace truck::simulator {

SimulatorMap::SimulatorMap(double precision, const geom::Vec2& base_to_lidar,
    const model::Lidar& lidar, const model::Shape& shape) {

    params_.precision = precision;
    initializeLidarParams(base_to_lidar, lidar);
    initializeShapeParams(shape);
}

void SimulatorMap::initializeLidarParams(const geom::Vec2& base_to_lidar,
    const model::Lidar& lidar) {

    params_.base_to_lidar = base_to_lidar;

    const double angle_min_rad = lidar.angle_min.radians();
    const double angle_max_rad = lidar.angle_max.radians();
    const double angle_increment_rad = lidar.angle_increment.radians();

    params_.lidar_rays_number = (angle_max_rad - angle_min_rad) / angle_increment_rad;
    params_.lidar_angle_min = geom::AngleVec2(lidar.angle_min);
    params_.lidar_angle_increment = lidar.angle_increment;
}

void SimulatorMap::initializeShapeParams(const model::Shape& shape) {
    params_.shape_length = shape.length;
    params_.shape_width_half = shape.width / 2;
}

void SimulatorMap::initializeRTree() {
    RTreeIndexedSegments segments;
    segments.reserve(obstacles_.size());
    for (auto i = 0; i < obstacles_.size(); ++i) {
        const auto first = RTreePoint(obstacles_[i].begin.x, obstacles_[i].begin.y);
        const auto second = RTreePoint(obstacles_[i].end.x, obstacles_[i].end.y);
        segments.push_back({RTreeSegment(first, second), i});
    }

    rtree_ = RTree(segments.begin(), segments.end());
}

void SimulatorMap::resetMap(const std::string& path) {
    obstacles_.clear();
    const auto map = map::Map::fromGeoJson(path);
    const auto polygons = map.polygons();
    for (const auto &polygon: polygons) {
        auto segments = polygon.segments();
        obstacles_.insert(obstacles_.end(), 
            std::make_move_iterator(segments.begin()), 
            std::make_move_iterator(segments.end()));
    }

    initializeRTree();
}

void SimulatorMap::eraseMap() {
    obstacles_.clear();
    rtree_.clear();
}

namespace {

RTreeBox getBox(const geom::Vec2& rear_ax_center,
    double length, double width_half, double yaw) {

    const geom::Vec2 center(
        rear_ax_center.x + length / 2 * cos(yaw),
        rear_ax_center.y + length / 2 * sin(yaw));
    const double radius = length * 2;

    return {
        RTreePoint(center.x - radius, center.y - radius),
        RTreePoint(center.x + radius, center.y + radius)
    };
}

geom::Polygon getBounds(const geom::Vec2& rear_ax_center,
    double length, double width_half, double yaw) {

    const auto yaw_vec = geom::Vec2::fromAngle(geom::Angle::fromRadians(yaw));
    const auto dir = length * yaw_vec;
    const auto norm = width_half * yaw_vec;
    const geom::Vec2 a(rear_ax_center.x - norm.y, rear_ax_center.y + norm.x);
    const geom::Vec2 b(rear_ax_center.x + norm.y, rear_ax_center.y - norm.x);
    return {a, b, a + dir, b + dir};
}

} // namespace

bool SimulatorMap::checkForCollisions(const geom::Vec2& rear_ax_center, double yaw) const {
    const auto box = getBox(rear_ax_center,
        params_.shape_length, params_.shape_width_half, yaw);
    std::vector<RTreeIndexedSegment> query_result;
    std::vector<RTreeIndexedSegment> result;
    rtree_.query(bgi::intersects(box), std::back_inserter(result));

    const auto bounds = getBounds(rear_ax_center,
        params_.shape_length, params_.shape_width_half, yaw);
    for (const auto& rtreeSegment : result) {
        const geom::Segment segment(
            {
                bg::get<0, 0>(rtreeSegment.first),
                bg::get<0, 1>(rtreeSegment.first)
            }, 
            {
                bg::get<1, 0>(rtreeSegment.first),
                bg::get<1, 1>(rtreeSegment.first)
            });

        if (geom::intersect(bounds, segment, params_.precision)) {
            return true;
        }
    }

    return false;
}

namespace {

int mod(int number, int divider) {
    return (number % divider + divider) % divider;
}

geom::Vec2 getLidarOrigin(const geom::Pose& odom_base_pose, const geom::Vec2& from_base) {
    const auto dir_vec = odom_base_pose.dir.vec();
    const auto x = odom_base_pose.pos.x 
        + from_base.x * dir_vec.x - from_base.y * dir_vec.y;
    const auto y = odom_base_pose.pos.y
        + from_base.y * dir_vec.x + from_base.x * dir_vec.y;
    return geom::Vec2(x, y);
}

geom::Angle getOrientedAngle(const geom::Vec2& origin_to_point, 
    const geom::Vec2& dir) {

    auto origin_to_point_angle = geom::angleBetween(dir, origin_to_point);
    return origin_to_point_angle._0_2PI();
}

double ceilWithPrecision(double number, double precision) {
    return std::ceil(number / precision) * precision;
}

geom::AngleVec2 getRayDir(const geom::AngleVec2& zero_dir, 
    const geom::Angle increment, const int index) {
    
    const auto mult_increment = geom::AngleVec2(index * increment);
    return zero_dir + mult_increment;
}

float getIntersectionDistance(const geom::Ray& ray, 
    const geom::Segment& segment, double precision) {

    const auto intersection = geom::intersect(ray, segment, precision);
    if (!intersection) {
        return std::numeric_limits<float>::max();
    }

    const auto distance = geom::distance(*intersection, ray.origin);
    return static_cast<float>(distance);
}

} // namespace

std::vector<float> SimulatorMap::getLidarRanges(const geom::Pose& odom_base_pose) const {
    std::vector<float> ranges(params_.lidar_rays_number, std::numeric_limits<float>::max());
    
    const auto origin = getLidarOrigin(odom_base_pose, params_.base_to_lidar);
    const auto dir = (odom_base_pose.dir + params_.lidar_angle_min);
    const auto dir_vector = dir.vec();
    const auto increment_rad = params_.lidar_angle_increment.radians();

    for (const auto& segment : obstacles_) {
        const auto origin_begin = segment.begin - origin;
        const auto origin_end = segment.end - origin;

        auto begin_oriented_angle = getOrientedAngle(origin_begin, dir_vector);
        auto end_oriented_angle = getOrientedAngle(origin_end, dir_vector);

        const int sign = geom::angleBetween(origin_begin, origin_end).radians() > 0
            ? 1 : -1;
        int begin_index, end_index;

        if (sign > 0) {
            begin_index = ceilWithPrecision(
                begin_oriented_angle.radians() / increment_rad, 
                params_.precision);
            end_index = end_oriented_angle.radians() / increment_rad;
        } else {
            begin_index = begin_oriented_angle.radians() / increment_rad;
            end_index = ceilWithPrecision(
                end_oriented_angle.radians() / increment_rad, 
                params_.precision);
        }

        if (begin_index >= params_.lidar_rays_number
            && end_index >= params_.lidar_rays_number) {
            continue;
        }

        begin_index = std::min(begin_index, params_.lidar_rays_number - 1);
        end_index = std::min(end_index, params_.lidar_rays_number - 1);

        geom::Ray current_ray(origin, 
            getRayDir(dir, params_.lidar_angle_increment, begin_index));
        const auto increment = geom::AngleVec2(sign * params_.lidar_angle_increment);
        int index = begin_index - sign;
        
        do {
            index = mod(index + sign, params_.lidar_rays_number);
            const auto distance = getIntersectionDistance(current_ray, segment, params_.precision);
            ranges[index] = std::min(ranges[index], distance);
            current_ray.dir += increment;
        } while (index != end_index);
    }

    return ranges;
}

}  // namespace truck::simulator
