#pragma once

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "pose.hpp"
#include "math_helpers.hpp"

namespace robolab {
namespace aruco {

class Coordinator {
public:
    Coordinator(int marker_count);

    void update(const std::vector<int> &ids, const std::vector<cv::Vec3d> &rvecs, const std::vector<cv::Vec3d> &tvecs);

    void update(const std::vector<int> &ids, const std::vector<math::Transform> &transforms);

    Pose get_pose();

private:
    std::vector<std::optional<math::Transform>> to_anchor_transform_;

    Pose current_pose_;

    int anchor_id_ = 0;
};

}
}

