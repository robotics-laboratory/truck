#pragma once

#include "geom/pose.h"
#include "geom/transform.h"
#include "geom/vector.h"
#include "model/params.h"

#include <opencv2/core.hpp>

#include <optional>

namespace truck::collision_checker {

struct MapMeta {
    // origin is in the 'map' frame, dir is x axis of the 'map' frame
    geom::Pose origin;
    double resolution;
    uint32_t width;
    uint32_t height;
};

class StaticCollisionChecker {
  public:
    constexpr static double kMaxDistance = 10.0;

    StaticCollisionChecker(const model::Shape& shape);

    bool initialized() const;
    void reset(const MapMeta& meta, const cv::Mat& distance_transform);

    double distance(const geom::Pose& ego_pose) const;
    double distance(const geom::Vec2& point) const;

  private:
    model::Shape shape_;

    struct State {
        MapMeta meta;
        geom::Transform tf;
        cv::Mat distance_transform;
    };

    std::optional<State> state_;
};

}  // namespace truck::collision_checker