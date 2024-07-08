#include "geom/pose.h"
#include "geom/polygon.h"
#include "geom/segment.h"
#include "geom/boost/point.h"
#include "geom/boost/segment.h"
#include "model/model.h"

#include <boost/geometry.hpp>

#include <string>
#include <vector>

namespace truck::simulator {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using RTreePoint = geom::Vec2;
using RTreeSegment = geom::Segment;
using RTreeBox = bg::model::box<RTreePoint>;
using RTreeIndexedSegment = std::pair<RTreeSegment, size_t>;
using RTreeIndexedSegments = std::vector<RTreeIndexedSegment>;
using RTree = bgi::rtree<RTreeIndexedSegment, bgi::rstar<16>>;

class SimulationMap {
  public:
    void resetMap(const std::string& path);
    void eraseMap();
    const geom::Segments& obstacles() const noexcept { return obstacles_; }
    const RTree& rtree() const noexcept { return rtree_; }

  private:
    void initializeRTree();

    geom::Segments obstacles_;
    RTree rtree_;
};

bool hasCollision(const SimulationMap& map, const geom::Polygon& shape_polygon, double precision);
std::vector<float> getLidarRanges(
    const SimulationMap& map, const geom::Pose& lidar_pose, const model::Lidar& lidar,
    double precision);

}  // namespace truck::simulator
