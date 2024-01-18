#pragma once

#include "collision/collision_map.h"

#include "fastgrid/interpolation.h"

#include "geom/pose.h"
#include "geom/transform.h"
#include "geom/vector.h"
#include "model/params.h"

#include <opencv2/core.hpp>

#include <memory>
#include <optional>

namespace truck::collision {

class StaticCollisionChecker {
  public:
    constexpr static double kMaxDistance = 10.0;

    StaticCollisionChecker(const model::Shape& shape);

    bool initialized() const;
    void reset(const std::shared_ptr<CollisionMap> collision_map);

    double distance(const geom::Pose& ego_pose) const noexcept;
    double distance(const geom::Vec2& point) const noexcept;

  private:
    model::Shape shape_;

    struct State {
        std::shared_ptr<CollisionMap> collision_map = nullptr;
    } state_;
};

}  // namespace truck::collision
