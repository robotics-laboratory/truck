#pragma once

#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "pose.hpp"
#include "math/math_helpers.hpp"

namespace rosaruco {

class Coordinator {
public:
    Coordinator(int marker_count);

    void update(const std::vector<int> &ids, const std::vector<cv::Vec3d> &rvecs, const std::vector<cv::Vec3d> &tvecs);

    void update(const std::vector<int> &ids, const std::vector<Transform> &transforms);

    const std::optional<Transform>& get_transform_to_anchor(int from_id) const;

    Pose get_pose();

private:
    std::vector<std::optional<Transform>> to_anchor_transform_;

    Pose current_pose_;

    int anchor_id_ = 0;
};

}
