#pragma once

#include "collision/map.h"

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
    void reset(Map distance_transform);

    double distance(const geom::Pose& ego_pose) const;
    double distance(const geom::Vec2& point) const;

  private:
    model::Shape shape_;

    struct State {
        Map distance_transform;
        geom::Transform tf;
    };

    std::optional<State> state_ = std::nullopt;
};

}  // namespace truck::collision
