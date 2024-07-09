#include "geom/pose.h"
#include "geom/polygon.h"
#include "geom/segment.h"
#include "geom/boost/point.h"
#include "geom/boost/segment.h"
#include "geom/boost/box.h"
#include "model/model.h"

#include <boost/geometry.hpp>

#include <string>
#include <vector>

namespace truck::simulator {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using IndexSegment = std::pair<geom::Segment, size_t>;
using IndexSegments = std::vector<IndexSegment>;
using RTree = bgi::rtree<IndexSegment, bgi::rstar<16>>;

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
