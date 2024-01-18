#pragma once

#include "collision/map.h"

#include "fastgrid/interpolation.h"

#include "geom/pose.h"
#include "geom/transform.h"
#include "geom/vector.h"
#include "model/params.h"

#include <opencv2/core.hpp>

#include <optional>

namespace truck::collision {

class StaticCollisionChecker {
  public:
    constexpr static double kMaxDistance = 10.0;

    StaticCollisionChecker(const model::Shape& shape);

    bool initialized() const;
    void reset(const fastgrid::F32Grid& distance_map);

    double distance(const geom::Pose& ego_pose) const;
    double distance(const geom::Vec2& point) const;

  private:
    model::Shape shape_;

    struct State {
        std::optional<fastgrid::Bilinear<float>> distance_transform = std::nullopt;
    } state_;
};

}  // namespace truck::collision
