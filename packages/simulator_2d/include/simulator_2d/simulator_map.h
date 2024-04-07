#include "model/model.h"
#include "geom/pose.h"
#include "geom/segment.h"
#include "geom/vector.h"

#include <string>
#include <vector>

namespace truck::simulator {

class SimulatorMap {
  public:
    SimulatorMap(double precision, const geom::Vec2& base_to_lidar,
      const model::Lidar& lidar_config);

    void resetMap(const std::string& path);
    void eraseMap();
    bool checkForCollisions(const geom::Vec2& rear_ax_center,
      double length, double width_half, double yaw) const;
    std::vector<float> getLidarRanges(const geom::Pose& odom_base_pose) const;

  private:
    struct Parameters {
        double precision;
        geom::Vec2 base_to_lidar;
        int lidar_rays_number;
        geom::AngleVec2 lidar_angle_min;
        geom::Angle lidar_angle_increment;
    } params_;

    geom::Segments obstacles_;
};

}  // namespace truck::simulator
