#include "simulator_2d/simulation_map.h"

#include "map/map.h"
#include "geom/bounding_box.h"
#include "geom/distance.h"
#include "geom/intersection.h"
#include "geom/ray.h"
#include "geom/vector.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace truck::simulator {

void SimulationMap::initializeRTree() {
    RTreeIndexedSegments segments;
    segments.reserve(obstacles_.size());
    for (auto i = 0; i < obstacles_.size(); ++i) {
        const auto first = RTreePoint(obstacles_[i].begin.x, obstacles_[i].begin.y);
        const auto second = RTreePoint(obstacles_[i].end.x, obstacles_[i].end.y);
        segments.push_back({RTreeSegment(first, second), i});
    }

    rtree_ = RTree(segments.begin(), segments.end());
}

void SimulationMap::resetMap(const std::string& path) {
    eraseMap();
    const auto map = map::Map::fromGeoJson(path);
    const auto polygons = map.polygons();
    for (const auto& polygon : polygons) {
        auto segments = polygon.segments();
        obstacles_.insert(
            obstacles_.end(),
            std::make_move_iterator(segments.begin()),
            std::make_move_iterator(segments.end()));
    }

    initializeRTree();
}

void SimulationMap::eraseMap() {
    obstacles_.clear();
    rtree_.clear();
}

bool hasCollision(const SimulationMap& map, const geom::Polygon& shape_polygon, double precision) {
    const auto bounding_box = geom::makeBoundingBox(shape_polygon);
    const RTreeBox rtree_box = {
        RTreePoint(bounding_box.min.x, bounding_box.min.y),
        RTreePoint(bounding_box.max.x, bounding_box.max.y)};

    std::vector<RTreeIndexedSegment> result;
    map.rtree().query(bgi::intersects(rtree_box), std::back_inserter(result));

    for (const auto& rtreeSegment : result) {
        const geom::Segment segment(
            {bg::get<0, 0>(rtreeSegment.first), bg::get<0, 1>(rtreeSegment.first)},
            {bg::get<1, 0>(rtreeSegment.first), bg::get<1, 1>(rtreeSegment.first)});

        if (geom::intersect(shape_polygon, segment, precision)) {
            return true;
        }
    }

    return false;
}

namespace {

geom::Angle getOrientedAngle(const geom::Vec2& origin_to_point, const geom::Vec2& dir) {
    auto origin_to_point_angle = geom::angleBetween(dir, origin_to_point);
    return origin_to_point_angle._0_2PI();
}

double ceilWithPrecision(double number, double precision) {
    return std::ceil(number / precision) * precision;
}

geom::AngleVec2 getRayDir(
    const geom::AngleVec2& zero_dir, const geom::Angle increment, const int index) {
    const auto mult_increment = geom::AngleVec2(index * increment);
    return zero_dir + mult_increment;
}

float getIntersectionDistance(
    const geom::Ray& ray, const geom::Segment& segment, double precision) {
    const auto intersection = geom::intersect(ray, segment, precision);
    if (!intersection) {
        return std::numeric_limits<float>::max();
    }

    const auto distance = geom::distance(*intersection, ray.origin);
    return static_cast<float>(distance);
}

int mod(int number, int divider) { return (number % divider + divider) % divider; }

}  // namespace

std::vector<float> getLidarRanges(
    const SimulationMap& map, const geom::Pose& lidar_pose, const model::Lidar& lidar, double precision) {
    const double angle_min_rad = lidar.angle_min.radians();
    const double angle_max_rad = lidar.angle_max.radians();
    const auto lidar_angle_increment = lidar.angle_increment;
    const auto increment_rad = lidar_angle_increment.radians();
    const int lidar_rays_number = (angle_max_rad - angle_min_rad) / increment_rad;
    const auto lidar_angle_min = geom::AngleVec2(lidar.angle_min);
    const auto dir = (lidar_pose.dir + lidar_angle_min);
    const auto dir_vector = dir.vec();

    std::vector<float> ranges(lidar_rays_number, std::numeric_limits<float>::max());

    for (const auto& segment : map.obstacles()) {
        const auto origin_begin = segment.begin - lidar_pose;
        const auto origin_end = segment.end - lidar_pose;

        auto begin_oriented_angle = getOrientedAngle(origin_begin, dir_vector);
        auto end_oriented_angle = getOrientedAngle(origin_end, dir_vector);

        const int sign = geom::angleBetween(origin_begin, origin_end).radians() > 0 ? 1 : -1;
        int begin_index, end_index;

        if (sign > 0) {
            begin_index = ceilWithPrecision(
                begin_oriented_angle.radians() / increment_rad, precision);
            end_index = end_oriented_angle.radians() / increment_rad;
        } else {
            begin_index = begin_oriented_angle.radians() / increment_rad;
            end_index =
                ceilWithPrecision(end_oriented_angle.radians() / increment_rad, precision);
        }

        if (begin_index >= lidar_rays_number && end_index >= lidar_rays_number) {
            continue;
        }

        begin_index = std::min(begin_index, lidar_rays_number - 1);
        end_index = std::min(end_index, lidar_rays_number - 1);

        geom::Ray current_ray(lidar_pose, getRayDir(dir, lidar_angle_increment, begin_index));
        const auto increment = geom::AngleVec2(sign * lidar_angle_increment);
        int index = begin_index - sign;

        do {
            index = mod(index + sign, lidar_rays_number);
            const auto distance = getIntersectionDistance(current_ray, segment, precision);
            ranges[index] = std::min(ranges[index], distance);
            current_ray.dir += increment;
        } while (index != end_index);
    }

    return ranges;
}

}  // namespace truck::simulator
