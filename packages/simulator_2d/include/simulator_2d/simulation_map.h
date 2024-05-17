#include "geom/pose.h"
#include "geom/polygon.h"
#include "geom/segment.h"
#include "model/model.h"

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

class SimulationMap {
  public:
    SimulationMap(double precision = 1e-8);

    void resetMap(const std::string& path);
    void eraseMap();
    bool checkForCollisions(const geom::Polygon& shape_polygon) const;
    std::vector<float> getLidarRanges(const geom::Pose& lidar_pose, const model::Lidar& lidar) const;

  private:
    void initializeRTree();

    struct Parameters {
        double precision;
    } params_;

    geom::Segments obstacles_;
    RTree rtree_;
};

}  // namespace truck::simulator
