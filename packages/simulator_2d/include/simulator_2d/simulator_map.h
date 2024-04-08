#include "model/model.h"
#include "geom/pose.h"
#include "geom/segment.h"
#include "geom/vector.h"

#include <boost/geometry.hpp>

#include <string>
#include <vector>

namespace truck::simulator {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using RTreePoint = bg::model::point<double, 2, bg::cs::cartesian>;
using RTreeSegment = bg::model::segment<RTreePoint>;
using RTreeBox = bg::model::box<RTreePoint>;
using RTreeIndexedSegment = std::pair<RTreeSegment, size_t>;
using RTreeIndexedSegments = std::vector<RTreeIndexedSegment>;
using RTree = bgi::rtree<RTreeIndexedSegment, bgi::rstar<16>>;

class SimulatorMap {
  public:
    SimulatorMap(double precision, const geom::Vec2& base_to_lidar,
      const model::Lidar& lidar, const model::Shape& shape);

    void resetMap(const std::string& path);
    void eraseMap();
    bool checkForCollisions(const geom::Vec2& rear_ax_center, double yaw) const;
    std::vector<float> getLidarRanges(const geom::Pose& odom_base_pose) const;

  private:
    void initializeLidarParams(const geom::Vec2& base_to_lidar,
      const model::Lidar& lidar);
    void initializeShapeParams(const model::Shape& shape);
    void initializeRTree();

    struct Parameters {
        double precision;
        geom::Vec2 base_to_lidar;
        int lidar_rays_number;
        geom::AngleVec2 lidar_angle_min;
        geom::Angle lidar_angle_increment;
        double shape_length;
        double shape_width_half;
    } params_;

    geom::Segments obstacles_;
    RTree rtree_;
};

}  // namespace truck::simulator
