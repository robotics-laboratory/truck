#include "simulator_2d/simulator_map.h"

#include "map/map.h"
#include "geom/distance.h"
#include "geom/intersection.h"

#include <boost/geometry.hpp>

#include <cmath>
#include <limits>

namespace truck::simulator {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using RTreePoint = bg::model::point<double, 2, bg::cs::cartesian>;
using RTreeSegment = bg::model::segment<RTreePoint>;
using RTreeBox = bg::model::box<RTreePoint>;
using RTreeIndexedPoint = std::pair<RTreeSegment, size_t>;
using RTreeIndexedPoints = std::vector<RTreeIndexedPoint>;
using RTree = bgi::rtree<RTreeIndexedPoint, bgi::rstar<16>>;

SimulatorMap::SimulatorMap(double precision, const geom::Vec2& base_to_lidar,
    const model::Lidar& lidar_config) {

    params_.precision = precision;
    initializeLidarParams(base_to_lidar, lidar_config);
}

void SimulatorMap::initializeLidarParams(const geom::Vec2& base_to_lidar,
    const model::Lidar& lidar_config) {

    params_.base_to_lidar = base_to_lidar;

    const double angle_min_rad = lidar_config.angle_min.radians();
    const double angle_max_rad = lidar_config.angle_max.radians();
    const double angle_increment_rad = lidar_config.angle_increment.radians();

    params_.lidar_rays_number = (angle_max_rad - angle_min_rad) / angle_increment_rad;
    params_.lidar_angle_min = geom::AngleVec2(lidar_config.angle_min);
    params_.lidar_angle_increment = lidar_config.angle_increment;
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
}

void SimulatorMap::eraseMap() {
    obstacles_.clear();
}

namespace {

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

void SimulatorMap::checkForCollisions(const geom::Vec2& rear_ax_center,
    double length, double width_half, double yaw) {

    const geom::Polygon bounds = getBounds(rear_ax_center, length, width_half, yaw);
    for (const auto& segment : obstacles_) {
        if (geom::intersect(bounds, segment, params_.precision)) {
            fail_ = true;
            return;
        }
    }
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
