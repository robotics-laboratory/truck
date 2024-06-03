#pragma once

#include <deque>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "math_helpers.hpp"
#include "pose.hpp"
#include "tf_graph.hpp"

namespace rosaruco {

class CameraTracker {
  public:
    CameraTracker(int markers_count_);

    void Update(const std::vector<int>& ids, const std::vector<Transform>& from_marker_to_cam);

    const std::optional<Transform>& GetTransformToAnchor(int from_id) const;

    Pose GetPose();

  private:
    TfGraph graph_;

    std::vector<std::optional<Transform>> to_anchor_;

    Pose current_pose_;

    int anchor_id_ = -1;
};

}  // namespace rosaruco
