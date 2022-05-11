#pragma once

#include <deque>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "pose.hpp"
#include "math/math_helpers.hpp"

namespace rosaruco {

class Coordinator {
public:
    Coordinator(int marker_count);

    void Update(const std::vector<int> &ids, const std::vector<cv::Vec3d> &rvecs, const std::vector<cv::Vec3d> &tvecs);

    void Update(const std::vector<int> &ids, const std::vector<Transform> &transforms);

    const std::optional<Transform>& GetTransformToAnchor(int from_id) const;

    Pose GetPose();

    void SetAnchorId(int id);

private:

    class Edge {
    public:
        Edge();
        const Transform& GetAverage() const ;
        double GetError() const;
        void AddTransform(const Transform& t);
        bool Empty() const;
    private:
        cv::Mat quaternion_sum_;
        tf2::Vector3 average_translation_;
        int transforms_count_;

        Transform average_transform_;
        double error_;
    };

    int nodes_count_;
    std::vector<std::vector<Edge>> edges_;

    std::vector<std::optional<Transform>> to_anchor_transform_;

    Pose current_pose_;

    int anchor_id_ = -1;
};

}
