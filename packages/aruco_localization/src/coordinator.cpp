#include "coordinator.hpp"
#include "math/math_helpers.hpp"

#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace rosaruco {

Coordinator::Coordinator(int marker_count) {
    to_anchor_transform_.resize(marker_count);
    to_anchor_transform_[anchor_id_] = Transform(tf2::Quaternion(0, 0, 0, 1), {0, 0, 0});
}

void Coordinator::update(const std::vector<int> &ids, const std::vector<cv::Vec3d> &rvecs, const std::vector<cv::Vec3d> &tvecs) {
    std::vector<Transform> transforms;
    transforms.reserve(rvecs.size());

    for (size_t i = 0; i < ids.size(); i++) {
        transforms.push_back(GetTransform(rvecs[i], tvecs[i]));
    }

    update(ids, transforms);
}


void Coordinator::update(const std::vector<int> &ids, const std::vector<Transform> &transforms) {
    for (size_t i = 0; i < ids.size(); i++) {
        for (size_t j = 0; j < ids.size(); j++) {
            if (!to_anchor_transform_[ids[i]] && to_anchor_transform_[ids[j]]) {
                to_anchor_transform_[ids[i]] = *to_anchor_transform_[ids[j]] * transforms[j].inverse() * transforms[i];
            }
        }
    }

    Pose new_pose{tf2::Quaternion(0, 0, 0, 0), tf2::Vector3(0, 0, 0)};
    int anchored_count = 0;

    for (size_t i = 0; i < ids.size(); i++) {
        if (to_anchor_transform_[ids[i]]) {
            anchored_count++;
            auto from_cam_to_anchor = transforms[i].inverse() * *to_anchor_transform_[ids[i]];
            new_pose.point += from_cam_to_anchor({0, 0, 0});
            new_pose.orientation += from_cam_to_anchor.getRotation();
        }
    }

    new_pose.orientation *= tf2::Quaternion(tf2::Vector3(0, 1, 0), -M_PI / 2) 
                * tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI / 2);
    
    new_pose.point /= anchored_count;
    new_pose.orientation /= anchored_count;

    current_pose_ = new_pose;
}

const std::optional<Transform>& Coordinator::get_transform_to_anchor(int from_id) const {
    return to_anchor_transform_[from_id];
}

Pose Coordinator::get_pose() {
    return current_pose_;
}

}